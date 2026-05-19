#pragma once
#include <string>
#include <stdint.h>
#include <ctime>
#include <utils/new_event.h>

// POCSAG sync word (32 bits)
#define POCSAG_FRAME_SYNC_CODEWORD  ((uint32_t)0x7CD215D8U)
// Inverted POCSAG sync word for negative-polarity signals
#define POCSAG_FRAME_SYNC_INVERTED  ((uint32_t)0x832DEA27U)
// Idle data field (21 bits before BCH)
#define POCSAG_IDLE_CODEWORD        ((uint32_t)0x7A89C197U)
// Maximum Hamming distance for sync acquisition
#define POCSAG_SYNC_DIST            3
// Each batch has 16 codewords (8 frames of 2 codewords each)
#define POCSAG_BATCH_CODEWORD_COUNT 16

namespace pocsag {
    enum CodewordType {
        CODEWORD_TYPE_IDLE      = -1,
        CODEWORD_TYPE_ADDRESS   = 0,
        CODEWORD_TYPE_MESSAGE   = 1
    };

    // Possible decoded message types
    enum MessageType {
        MESSAGE_TYPE_TONE_ONLY      = 0,    // No content (tone-only page)
        MESSAGE_TYPE_NUMERIC        = 1,    // 4-bit per char numeric
        MESSAGE_TYPE_ALPHANUMERIC   = 2     // 7-bit per char ASCII
    };

    // User-selectable decode mode
    enum DecodeMode {
        DECODE_MODE_AUTO        = 0,    // Use function bits (ITU-R M.584-2)
        DECODE_MODE_NUMERIC     = 1,    // Force numeric decoding
        DECODE_MODE_ALPHA       = 2     // Force alphanumeric decoding
    };

    using Codeword = uint32_t;
    using Address  = uint32_t;

    // Decoded message reported to the application
    struct Message {
        std::time_t timestamp;      // System time at flush
        Address     address;        // 21-bit CAPCODE
        uint8_t     function;       // Function bits (0-3)
        MessageType type;           // Decoded message type
        std::string content;        // Decoded text
        int         corrected;      // Number of bits corrected by BCH
        int         errors;         // Number of uncorrectable codewords
    };

    class Decoder {
    public:
        Decoder();

        // Process a stream of hard-decision symbols (0 / 1)
        void process(uint8_t* symbols, int count);

        // Set the user-selectable decoding behaviour
        void setDecodeMode(DecodeMode mode) { _decodeMode = mode; }

        // Whether the input polarity is inverted (high freq = 1)
        void setInverted(bool inv) { _inverted = inv; }

        // Reset the internal state machine (sync, batch, partial message)
        void reset();

        // Emitted whenever a complete message has been decoded
        NewEvent<const Message&> onMessage;

    private:
        static int  distance(uint32_t a, uint32_t b);
        bool        correctCodeword(Codeword in, Codeword& out, int& correctedBits);
        static uint32_t bchSyndrome(uint32_t cw31);
        static bool checkParity(uint32_t cw);
        void        flushMessage();
        void        decodeBatch();

        // Sync detection
        uint32_t syncSR     = 0;
        bool     synced     = false;
        int      batchOffset = 0;

        // Current batch accumulation
        Codeword batch[POCSAG_BATCH_CODEWORD_COUNT];

        // Current message being assembled across codewords
        Address     addr        = 0;
        uint8_t     function    = 0;
        MessageType msgType     = MESSAGE_TYPE_ALPHANUMERIC;
        std::string msg;
        int         msgCorrected = 0;
        int         msgErrors    = 0;
        int         msgCwReceived = 0;   // Number of MESSAGE codewords decoded for this message
        bool        msgActive    = false;

        // Partial character state for alphanumeric decoding
        char currChar   = 0;
        int  currOffset = 0;

        // User options
        DecodeMode _decodeMode = DECODE_MODE_AUTO;
        bool       _inverted   = false;
    };
}
