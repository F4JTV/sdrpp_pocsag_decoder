#include "pocsag.h"
#include <string.h>
#include <utils/flog.h>

// BCH(31,21) generator polynomial g(x) = x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
// Binary: 11101101001 = 0x769
#define POCSAG_GEN_POLY             ((uint32_t)0x769U)
#define POCSAG_BATCH_BIT_COUNT      (POCSAG_BATCH_CODEWORD_COUNT * 32)

namespace pocsag {

    // POCSAG-2 numeric character set (ITU-R M.584-2)
    // 4-bit nibbles -> printable character
    static const char NUMERIC_CHARSET[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', '*', 'U', ' ', '-', ')', '('
    };

    Decoder::Decoder() {
        memset(batch, 0, sizeof(batch));
    }

    void Decoder::reset() {
        syncSR = 0;
        synced = false;
        batchOffset = 0;
        memset(batch, 0, sizeof(batch));

        msg.clear();
        currChar = 0;
        currOffset = 0;
        msgActive = false;
        msgCorrected = 0;
        msgErrors = 0;
        msgCwReceived = 0;
    }

    // Hamming distance between two 32-bit values
    int Decoder::distance(uint32_t a, uint32_t b) {
        uint32_t diff = a ^ b;
        int dist = 0;
        for (int i = 0; i < 32; i++) {
            dist += (diff >> i) & 1U;
        }
        return dist;
    }

    // Compute BCH(31,21) syndrome over the 31 most-significant bits
    // (bits 31..1 of the 32-bit codeword) by polynomial long division.
    uint32_t Decoder::bchSyndrome(uint32_t cw) {
        // Work with bits 31..1 in positions 30..0
        uint64_t reg = (uint64_t)(cw >> 1) & 0x7FFFFFFFULL;
        for (int i = 30; i >= 10; i--) {
            if (reg & (1ULL << i)) {
                reg ^= ((uint64_t)POCSAG_GEN_POLY << (i - 10));
            }
        }
        return (uint32_t)(reg & 0x3FFU);
    }

    // Even-parity check over all 32 bits
    bool Decoder::checkParity(uint32_t cw) {
        int count = 0;
        for (int i = 0; i < 32; i++) {
            count += (cw >> i) & 1U;
        }
        return (count & 1) == 0;
    }

    // Correct up to 2 bit errors in a 32-bit codeword.
    // Returns true if codeword is valid (possibly after correction).
    // correctedBits returns the number of bit flips performed.
    bool Decoder::correctCodeword(Codeword in, Codeword& out, int& correctedBits) {
        // First, try the parity-only short-circuit:
        // If the BCH syndrome of the 31 protected bits is zero we just need
        // the even-parity bit to also match.
        uint32_t syn = bchSyndrome(in);
        if (syn == 0) {
            if (checkParity(in)) {
                out = in;
                correctedBits = 0;
                return true;
            }
            // Single bit error in the parity bit
            out = in ^ 0x1U;
            correctedBits = 1;
            return true;
        }

        // Single-bit error in the BCH-protected portion
        for (int i = 1; i < 32; i++) {
            uint32_t test = in ^ (1U << i);
            if (bchSyndrome(test) == 0 && checkParity(test)) {
                out = test;
                correctedBits = 1;
                return true;
            }
        }

        // Two-bit error in the BCH-protected portion
        // BCH(31,21) can correct up to 2 errors
        for (int i = 1; i < 32; i++) {
            for (int j = i + 1; j < 32; j++) {
                uint32_t test = in ^ (1U << i) ^ (1U << j);
                if (bchSyndrome(test) == 0 && checkParity(test)) {
                    out = test;
                    correctedBits = 2;
                    return true;
                }
            }
        }

        // Two-bit error including the parity bit
        for (int i = 1; i < 32; i++) {
            uint32_t test = in ^ (1U << i) ^ 0x1U;
            if (bchSyndrome(test) == 0 && checkParity(test)) {
                out = test;
                correctedBits = 2;
                return true;
            }
        }

        // Uncorrectable
        out = in;
        correctedBits = 0;
        return false;
    }

    void Decoder::flushMessage() {
        if (!msgActive) { return; }

        Message m;
        m.timestamp = std::time(nullptr);
        m.address   = addr;
        m.function  = function;
        m.type      = msgType;
        m.content   = msg;
        m.corrected = msgCorrected;
        m.errors    = msgErrors;

        // Auto-trim trailing nulls / control chars from alphanumeric messages
        while (!m.content.empty()) {
            unsigned char c = (unsigned char)m.content.back();
            if (c == 0 || c == 0x04) { m.content.pop_back(); }
            else { break; }
        }

        // If we never received an actual MESSAGE codeword, this is either
        // a legitimate tone-only page or (more commonly at low SNR) a
        // false-positive address codeword decoded out of noise.
        // Reclassify so the user can filter these out in one toggle.
        if (msgCwReceived == 0) {
            m.type    = MESSAGE_TYPE_TONE_ONLY;
            m.content.clear();
        }

        onMessage(m);

        // Reset partial state
        msg.clear();
        currChar      = 0;
        currOffset    = 0;
        msgActive     = false;
        msgCorrected  = 0;
        msgErrors     = 0;
        msgCwReceived = 0;
    }

    // Reverse the 7 LSBs of a byte (POCSAG sends LSB first)
    static inline char bitswap7(char in) {
        char out = 0;
        for (int i = 0; i < 7; i++) {
            out |= ((in >> (6 - i)) & 1) << i;
        }
        return out;
    }

    void Decoder::process(uint8_t* symbols, int count) {
        for (int i = 0; i < count; i++) {
            // Apply inversion if requested
            uint32_t s = _inverted ? (1U - (symbols[i] & 1U)) : (uint32_t)(symbols[i] & 1U);

            if (!synced) {
                // Shift incoming bits into the sync register
                syncSR = (syncSR << 1) | s;
                if (distance(syncSR, POCSAG_FRAME_SYNC_CODEWORD) <= POCSAG_SYNC_DIST) {
                    synced = true;
                    batchOffset = 0;
                    memset(batch, 0, sizeof(batch));
                }
                continue;
            }

            // Pack symbol into the current batch (MSB first)
            batch[batchOffset >> 5] |= (s << (31 - (batchOffset & 0b11111)));
            batchOffset++;

            // End of batch -> decode and re-acquire sync
            if (batchOffset >= POCSAG_BATCH_BIT_COUNT) {
                decodeBatch();
                batchOffset = 0;
                synced = false;
                syncSR = 0;
                memset(batch, 0, sizeof(batch));
            }
        }
    }

    void Decoder::decodeBatch() {
        int consecutiveBad = 0;

        for (int i = 0; i < POCSAG_BATCH_CODEWORD_COUNT; i++) {
            Codeword cw = batch[i];
            int correctedBits = 0;

            // BCH error correction + parity check
            bool ok = correctCodeword(cw, cw, correctedBits);
            if (!ok) {
                // Uncorrectable codeword
                consecutiveBad++;
                if (msgActive) {
                    msgErrors++;
                    // End the current message after 2 consecutive bad codewords
                    if (consecutiveBad >= 2) {
                        flushMessage();
                    }
                }
                continue;
            }
            consecutiveBad = 0;
            msgCorrected += correctedBits;

            // Identify codeword type (top bit) and idle pattern
            CodewordType type = (CodewordType)((cw >> 31) & 1U);
            if (type == CODEWORD_TYPE_ADDRESS && cw == POCSAG_IDLE_CODEWORD) {
                type = CODEWORD_TYPE_IDLE;
            }

            if (type == CODEWORD_TYPE_IDLE) {
                // Idle -> flush any in-progress message
                flushMessage();
            }
            else if (type == CODEWORD_TYPE_ADDRESS) {
                // Address codeword starts a new message; flush any prior one
                flushMessage();

                // Extract the 18 address bits (cw[30..13]) and the
                // 3 frame-position bits (i/2) to form the 21-bit CAPCODE.
                addr = ((cw >> 13) & 0x3FFFFU) << 3;
                addr |= (uint32_t)(i >> 1) & 0x7U;

                // Function bits (cw[12..11])
                function = (uint8_t)((cw >> 11) & 0x3U);

                // Decide on the decoding mode for the upcoming message words
                switch (_decodeMode) {
                    case DECODE_MODE_NUMERIC:
                        msgType = MESSAGE_TYPE_NUMERIC;
                        break;
                    case DECODE_MODE_ALPHA:
                        msgType = MESSAGE_TYPE_ALPHANUMERIC;
                        break;
                    case DECODE_MODE_AUTO:
                    default:
                        // ITU-R M.584-2: function 0 is numeric,
                        // function 3 is alphanumeric, function 1/2 are
                        // commonly tone or alphanumeric depending on system.
                        if (function == 0) {
                            msgType = MESSAGE_TYPE_NUMERIC;
                        } else {
                            msgType = MESSAGE_TYPE_ALPHANUMERIC;
                        }
                        break;
                }

                msg.clear();
                currChar      = 0;
                currOffset    = 0;
                msgActive     = true;
                msgCwReceived = 0;
            }
            else if (type == CODEWORD_TYPE_MESSAGE) {
                if (!msgActive) {
                    // Stray message codeword (no preceding address) - skip
                    continue;
                }
                msgCwReceived++;

                // 20-bit payload from cw[30..11]
                uint32_t data = (cw >> 11) & 0xFFFFFU;

                if (msgType == MESSAGE_TYPE_NUMERIC) {
                    // 5 numeric chars per word, 4 bits each.
                    // Each nibble is sent LSB first, so we reverse it.
                    for (int n = 0; n < 5; n++) {
                        uint32_t shift = 16 - (n * 4);
                        uint32_t nib   = (data >> shift) & 0xFU;
                        uint32_t rev   = 0;
                        for (int b = 0; b < 4; b++) {
                            rev |= ((nib >> b) & 1U) << (3 - b);
                        }
                        msg += NUMERIC_CHARSET[rev & 0xFU];
                    }
                }
                else {
                    // Alphanumeric: 7-bit ASCII packed MSB-first into the
                    // 20-bit payload, character bits stored LSB first.
                    for (int b = 19; b >= 0; b--) {
                        currChar |= ((data >> b) & 1U) << currOffset;
                        currOffset++;
                        if (currOffset >= 7) {
                            char c = currChar & 0x7F;
                            // Skip NULs and EOT but keep other 7-bit chars
                            if (c != 0 && c != 0x04) { msg += c; }
                            currChar   = 0;
                            currOffset = 0;
                        }
                    }
                }
            }
        }
        // Note: a message that continues into the next batch will be flushed
        // either by the next address codeword, an idle codeword, or upon
        // re-sync at the end of the next batch.
    }
}
