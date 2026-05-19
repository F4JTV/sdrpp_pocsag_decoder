#pragma once
#include "../decoder.h"
#include <signal_path/vfo_manager.h>
#include <utils/optionlist.h>
#include <utils/flog.h>
#include <gui/widgets/symbol_diagram.h>
#include <gui/style.h>
#include <dsp/buffer/reshaper.h>
#include <dsp/sink/handler_sink.h>
#include <imgui.h>
#include <functional>
#include <cstring>
#include <algorithm>
#include "dsp.h"
#include "pocsag.h"

// Fixed VFO output sample rate. 24 kHz comfortably covers the +/-4.5 kHz
// deviation plus the highest POCSAG symbol rate (2400 baud) and gives a
// good number of samples per symbol at all three rates:
//    24000 / 512  = ~46.9 sps
//    24000 / 1200 =   20  sps
//    24000 / 2400 =   10  sps
#define POCSAG_VFO_SAMPLERATE   24000.0
#define POCSAG_VFO_BANDWIDTH    12500.0

// Number of symbols shown in the diagram and Reshaper output block
#define POCSAG_DIAG_BLOCK       1024

class POCSAGDecoder : public Decoder {
public:
    // Callback invoked from the DSP thread whenever a message is decoded.
    using MessageCallback = std::function<void(const pocsag::Message&)>;

    POCSAGDecoder(const std::string& name, VFOManager::VFO* vfo, MessageCallback onMessage)
        : diag(0.6f, POCSAG_DIAG_BLOCK)
    {
        this->name        = name;
        this->vfo         = vfo;
        this->onMessageCb = std::move(onMessage);

        // Define baudrate options
        baudrates.define(512,  "512 Baud",  512);
        baudrates.define(1200, "1200 Baud", 1200);
        baudrates.define(2400, "2400 Baud", 2400);

        // Define decode-mode options
        decodeModes.define(pocsag::DECODE_MODE_AUTO,    "Auto",          pocsag::DECODE_MODE_AUTO);
        decodeModes.define(pocsag::DECODE_MODE_ALPHA,   "Alphanumeric",  pocsag::DECODE_MODE_ALPHA);
        decodeModes.define(pocsag::DECODE_MODE_NUMERIC, "Numeric",       pocsag::DECODE_MODE_NUMERIC);

        // Default to 1200 baud (most common)
        brId   = baudrates.keyId(1200);
        dmId   = decodeModes.keyId(pocsag::DECODE_MODE_AUTO);
        invert = false;

        // Configure the VFO
        vfo->setBandwidthLimits(POCSAG_VFO_BANDWIDTH, POCSAG_VFO_BANDWIDTH, true);
        vfo->setSampleRate(POCSAG_VFO_SAMPLERATE, POCSAG_VFO_BANDWIDTH);

        // Build the DSP chain
        double baud = (double)baudrates.value(brId);
        dsp.init(vfo->output, POCSAG_VFO_SAMPLERATE, baud);

        // Reshaper for the symbol diagram: produce overlapping blocks of
        // POCSAG_DIAG_BLOCK samples about 15 times per second worth of
        // symbol-rate data, so the diagram stays smooth at any baudrate.
        int reshapeStride = std::max(8, POCSAG_DIAG_BLOCK / 12);
        reshape.init(&dsp.soft, POCSAG_DIAG_BLOCK, reshapeStride - POCSAG_DIAG_BLOCK);

        dataHandler.init(&dsp.out,    _dataHandler, this);
        diagHandler.init(&reshape.out, _diagHandler, this);

        // Apply initial options to the protocol decoder
        decoder.setDecodeMode(decodeModes.value(dmId));
        decoder.setInverted(invert);
        dsp.setLowPass(lowPass);

        // Wire the protocol decoder's message event to our static handler
        decoder.onMessage.bind(&POCSAGDecoder::messageHandler, this);
    }

    ~POCSAGDecoder() override {
        stop();
    }

    void showMenu() override {
        // Baudrate selector
        ImGui::LeftLabel("Baudrate");
        ImGui::FillWidth();
        if (ImGui::Combo(("##pager_decoder_pocsag_br_" + name).c_str(),
                         &brId, baudrates.txt))
        {
            double newBaud = (double)baudrates.value(brId);
            dsp.setBaudrate(newBaud);
            decoder.reset();
            if (settingsCallback) { settingsCallback(); }
        }

        // Decode-mode selector
        ImGui::LeftLabel("Decode");
        ImGui::FillWidth();
        if (ImGui::Combo(("##pager_decoder_pocsag_dm_" + name).c_str(),
                         &dmId, decodeModes.txt))
        {
            decoder.setDecodeMode(decodeModes.value(dmId));
            if (settingsCallback) { settingsCallback(); }
        }

        // Inversion toggle (some networks use reversed FSK polarity)
        if (ImGui::Checkbox(("Invert FSK##pager_decoder_pocsag_inv_" + name).c_str(),
                            &invert))
        {
            decoder.setInverted(invert);
            decoder.reset();
            if (settingsCallback) { settingsCallback(); }
        }

        // Audio-bandwidth low-pass filter after the FM demodulator.
        // Modeled on the radio module's NFM "Low Pass" option, which is
        // known to help significantly at low SNR by removing high-frequency
        // noise before the matched filter and symbol decision.
        if (ImGui::Checkbox(("Low Pass##pager_decoder_pocsag_lpf_" + name).c_str(),
                            &lowPass))
        {
            dsp.setLowPass(lowPass);
            if (settingsCallback) { settingsCallback(); }
        }

        // Symbol-diagram visualisation of the soft decisions
        ImGui::FillWidth();
        diag.draw();
    }

    void setVFO(VFOManager::VFO* vfo) override {
        this->vfo = vfo;
        vfo->setBandwidthLimits(POCSAG_VFO_BANDWIDTH, POCSAG_VFO_BANDWIDTH, true);
        vfo->setSampleRate(POCSAG_VFO_SAMPLERATE, POCSAG_VFO_BANDWIDTH);
        dsp.setInput(vfo->output);
    }

    void start() override {
        dsp.start();
        reshape.start();
        dataHandler.start();
        diagHandler.start();
    }

    void stop() override {
        dsp.stop();
        reshape.stop();
        dataHandler.stop();
        diagHandler.stop();
    }

    // ----- Persistent settings (called from main.cpp) -----------------
    int  getBaudrate()   const { return baudrates.value(brId); }
    int  getDecodeMode() const { return (int)decodeModes.value(dmId); }
    bool getInverted()   const { return invert; }
    bool getLowPass()    const { return lowPass; }

    void setBaudrateFromConfig(int baud) {
        if (!baudrates.keyExists(baud)) { return; }
        brId = baudrates.keyId(baud);
        dsp.setBaudrate((double)baud);
        decoder.reset();
    }

    void setDecodeModeFromConfig(int mode) {
        pocsag::DecodeMode dm = (pocsag::DecodeMode)mode;
        if (!decodeModes.keyExists(dm)) { return; }
        dmId = decodeModes.keyId(dm);
        decoder.setDecodeMode(dm);
    }

    void setInvertedFromConfig(bool inv) {
        invert = inv;
        decoder.setInverted(inv);
        decoder.reset();
    }

    void setLowPassFromConfig(bool lp) {
        lowPass = lp;
        dsp.setLowPass(lp);
    }

    // Settings-changed callback (main.cpp uses this to persist to JSON)
    void onSettingsChanged(std::function<void()> cb) {
        settingsCallback = std::move(cb);
    }

private:
    // DSP-thread handler: feed hard decisions into the protocol decoder
    static void _dataHandler(uint8_t* data, int count, void* ctx) {
        POCSAGDecoder* _this = (POCSAGDecoder*)ctx;
        _this->decoder.process(data, count);
    }

    // DSP-thread handler: push soft decisions into the symbol diagram
    static void _diagHandler(float* data, int count, void* ctx) {
        POCSAGDecoder* _this = (POCSAGDecoder*)ctx;
        float* buf = _this->diag.acquireBuffer();
        int n = std::min(count, POCSAG_DIAG_BLOCK);
        std::memcpy(buf, data, (size_t)n * sizeof(float));
        _this->diag.releaseBuffer();
    }

    // Forward decoded messages to the user-provided callback
    void messageHandler(const pocsag::Message& m) {
        flog::info("POCSAG: addr={} func={} type={} corrected={} errors={} msg='{}'",
                   (uint32_t)m.address,
                   (int)m.function,
                   (int)m.type,
                   m.corrected,
                   m.errors,
                   m.content);
        if (onMessageCb) { onMessageCb(m); }
    }

    std::string         name;
    VFOManager::VFO*    vfo = nullptr;

    POCSAGDSP                          dsp;
    dsp::buffer::Reshaper<float>       reshape;
    dsp::sink::Handler<uint8_t>        dataHandler;
    dsp::sink::Handler<float>          diagHandler;

    pocsag::Decoder                    decoder;
    ImGui::SymbolDiagram               diag;

    // UI state
    int  brId    = 0;
    int  dmId    = 0;
    bool invert  = false;
    bool lowPass = true;       // Default on - significant low-SNR benefit
    OptionList<int, int>                               baudrates;
    OptionList<pocsag::DecodeMode, pocsag::DecodeMode> decodeModes;

    MessageCallback           onMessageCb;
    std::function<void()>     settingsCallback;
};
