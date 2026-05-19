#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/main_window.h>
#include <gui/widgets/folder_select.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <utils/optionlist.h>
#include <utils/flog.h>

#include <ctime>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <fstream>
#include <algorithm>

#include "decoder.h"
#include "pocsag/decoder.h"

#define CONCAT(a, b) ((std::string(a) + b).c_str())

// Hard cap on the number of messages kept in memory for the GUI
#define POCSAG_MAX_MESSAGES_IN_RAM   1024

SDRPP_MOD_INFO{
    /* Name:            */ "pager_decoder",
    /* Description:     */ "POCSAG Pager Decoder (512 / 1200 / 2400 baud)",
    /* Author:          */ "SDR++ Community",
    /* Version:         */ 0, 2, 0,
    /* Max instances    */ -1
};

ConfigManager config;

class PagerDecoderModule : public ModuleManager::Instance {
public:
    PagerDecoderModule(std::string name)
        : logFolderSelect("%ROOT%")
    {
        this->name = name;

        // Define the snap-interval options shown in the UI combo
        snapIntervals.define(1,     "1 Hz",     1);
        snapIntervals.define(10,    "10 Hz",    10);
        snapIntervals.define(100,   "100 Hz",   100);
        snapIntervals.define(1000,  "1 kHz",    1000);
        snapIntervals.define(2500,  "2.5 kHz",  2500);
        snapIntervals.define(6250,  "6.25 kHz", 6250);
        snapIntervals.define(12500, "12.5 kHz", 12500);
        snapIntervals.define(25000, "25 kHz",   25000);
        snapId = snapIntervals.keyId(1000);   // default

        // Create the VFO with default POCSAG parameters
        vfo = sigpath::vfoManager.createVFO(name,
                                            ImGui::WaterfallVFO::REF_CENTER,
                                            0,        // offset
                                            12500,    // bandwidth
                                            24000,    // sample rate
                                            12500,    // bw lower limit
                                            12500,    // bw upper limit
                                            true);
        vfo->setSnapInterval(snapIntervals.value(snapId));

        // Build the POCSAG decoder, wiring its message callback to ours
        decoder = std::make_unique<POCSAGDecoder>(name, vfo,
            [this](const pocsag::Message& m) { this->onMessageReceived(m); });

        // Load persistent settings for this instance
        loadSettings();

        // Tell the decoder to save settings any time the user changes them
        decoder->onSettingsChanged([this]() { this->saveSettings(); });

        // Start the DSP chain
        decoder->start();

        // Register the menu entry
        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~PagerDecoderModule() {
        gui::menu.removeEntry(name);
        if (enabled) {
            decoder->stop();
            decoder.reset();
            sigpath::vfoManager.deleteVFO(vfo);
        }
    }

    void postInit() {}

    void enable() {
        double bw = gui::waterfall.getBandwidth();
        vfo = sigpath::vfoManager.createVFO(name,
                                            ImGui::WaterfallVFO::REF_CENTER,
                                            std::clamp<double>(0, -bw / 2.0, bw / 2.0),
                                            12500, 24000, 12500, 12500, true);
        vfo->setSnapInterval(snapIntervals.value(snapId));

        decoder = std::make_unique<POCSAGDecoder>(name, vfo,
            [this](const pocsag::Message& m) { this->onMessageReceived(m); });
        loadSettings();
        decoder->onSettingsChanged([this]() { this->saveSettings(); });
        decoder->start();
        enabled = true;
    }

    void disable() {
        if (!enabled) { return; }
        decoder->stop();
        decoder.reset();
        sigpath::vfoManager.deleteVFO(vfo);
        vfo = nullptr;
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    // -------- Settings persistence -----------------------------------
    void loadSettings() {
        config.acquire();
        if (!config.conf.contains(name)) {
            config.conf[name] = json({});
        }
        json& c = config.conf[name];

        if (c.contains("baudrate")) {
            decoder->setBaudrateFromConfig(c["baudrate"].get<int>());
        }
        if (c.contains("decodeMode")) {
            decoder->setDecodeModeFromConfig(c["decodeMode"].get<int>());
        }
        if (c.contains("invert")) {
            decoder->setInvertedFromConfig(c["invert"].get<bool>());
        }
        if (c.contains("lowPass")) {
            decoder->setLowPassFromConfig(c["lowPass"].get<bool>());
        }
        if (c.contains("snapInterval")) {
            int snap = c["snapInterval"].get<int>();
            if (snapIntervals.keyExists(snap)) {
                snapId = snapIntervals.keyId(snap);
                if (vfo) { vfo->setSnapInterval(snap); }
            }
        }
        if (c.contains("logToFile"))         { logToFile         = c["logToFile"].get<bool>(); }
        if (c.contains("logFolder"))         { logFolderSelect.setPath(c["logFolder"].get<std::string>()); }
        if (c.contains("hideErrorMessages")) { hideErrorMessages = c["hideErrorMessages"].get<bool>(); }
        if (c.contains("hideToneOnly"))      { hideToneOnly      = c["hideToneOnly"].get<bool>(); }

        config.release();
    }

    void saveSettings() {
        config.acquire();
        if (!config.conf.contains(name)) {
            config.conf[name] = json({});
        }
        config.conf[name]["baudrate"]          = decoder->getBaudrate();
        config.conf[name]["decodeMode"]        = decoder->getDecodeMode();
        config.conf[name]["invert"]            = decoder->getInverted();
        config.conf[name]["lowPass"]           = decoder->getLowPass();
        config.conf[name]["snapInterval"]      = snapIntervals.value(snapId);
        config.conf[name]["logToFile"]         = logToFile;
        config.conf[name]["logFolder"]         = logFolderSelect.path;
        config.conf[name]["hideErrorMessages"] = hideErrorMessages;
        config.conf[name]["hideToneOnly"]      = hideToneOnly;
        config.release(true);
    }

    // -------- Message handling ---------------------------------------
    // Called from the DSP thread when a complete message is decoded
    void onMessageReceived(const pocsag::Message& m) {
        {
            std::lock_guard<std::mutex> lck(messagesMtx);
            messages.push_back(m);
            while (messages.size() > POCSAG_MAX_MESSAGES_IN_RAM) {
                messages.pop_front();
            }
        }
        if (logToFile) { appendToLogFile(m); }
    }

    static std::string formatTimestamp(std::time_t t) {
        std::tm tm_buf;
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_buf);
        return buf;
    }

    static const char* typeName(pocsag::MessageType t) {
        switch (t) {
            case pocsag::MESSAGE_TYPE_NUMERIC:      return "Numeric";
            case pocsag::MESSAGE_TYPE_ALPHANUMERIC: return "Alpha";
            case pocsag::MESSAGE_TYPE_TONE_ONLY:    return "Tone";
            default:                                return "?";
        }
    }

    // Build the actual log file path from the user-selected folder.
    // We always write to "<folder>/pocsag_log.tsv" so the user only needs
    // to pick a directory (matching the Recorder module's pattern).
    // Not const because FolderSelect's accessors are not const.
    std::string logFilePath() {
        if (!logFolderSelect.pathIsValid() || logFolderSelect.path.empty()) {
            return "";
        }
        return logFolderSelect.expandString(logFolderSelect.path) + "/pocsag_log.tsv";
    }

    void appendToLogFile(const pocsag::Message& m) {
        std::string fp = logFilePath();
        if (fp.empty()) { return; }
        // Apply the same filters that the GUI applies, so the on-disk log
        // matches what the user actually sees
        if (hideErrorMessages && m.errors > 0)                            { return; }
        if (hideToneOnly       && m.type == pocsag::MESSAGE_TYPE_TONE_ONLY) { return; }
        std::ofstream f(fp, std::ios::app);
        if (!f.is_open()) { return; }
        f << formatTimestamp(m.timestamp)
          << '\t' << m.address
          << '\t' << (int)m.function
          << '\t' << typeName(m.type)
          << '\t' << m.content
          << '\n';
    }

    // -------- GUI ----------------------------------------------------
    static void menuHandler(void* ctx) {
        PagerDecoderModule* _this = (PagerDecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) { style::beginDisabled(); }

        // Protocol selector kept as a one-entry combo for forward
        // compatibility; only POCSAG is implemented in this module.
        ImGui::LeftLabel("Protocol");
        ImGui::FillWidth();
        const char* protoTxt = "POCSAG\0";
        int protoId = 0;
        ImGui::Combo(("##pager_decoder_proto_" + _this->name).c_str(), &protoId, protoTxt);

        // VFO snap interval (kHz step). Convenience for tuning to a known
        // pager channel grid.
        ImGui::LeftLabel("Snap");
        ImGui::FillWidth();
        if (ImGui::Combo(("##pager_decoder_snap_" + _this->name).c_str(),
                         &_this->snapId, _this->snapIntervals.txt))
        {
            if (_this->vfo) {
                _this->vfo->setSnapInterval(_this->snapIntervals.value(_this->snapId));
            }
            _this->saveSettings();
        }

        // Per-protocol menu
        if (_this->decoder) { _this->decoder->showMenu(); }

        ImGui::Separator();

        // Show/hide the messages window
        if (ImGui::Button(("Show Messages##pager_decoder_show_" + _this->name).c_str(),
                          ImVec2(menuWidth, 0)))
        {
            _this->showMessagesWindow = true;
        }

        // Log-to-file toggle + folder picker (like the Recorder module).
        // The file is named "pocsag_log.tsv" inside the selected folder.
        if (ImGui::Checkbox(("Log to file##pager_decoder_log_" + _this->name).c_str(),
                            &_this->logToFile))
        {
            _this->saveSettings();
        }
        if (_this->logToFile) {
            if (_this->logFolderSelect.render("##pager_decoder_logfolder_" + _this->name)) {
                _this->saveSettings();
            }
            if (_this->logFolderSelect.pathIsValid()) {
                ImGui::TextDisabled("File: pocsag_log.tsv");
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f),
                                   "Invalid folder");
            }
        }

        if (!_this->enabled) { style::endDisabled(); }

        // Detached messages window
        if (_this->showMessagesWindow) {
            _this->drawMessagesWindow();
        }
    }

    void drawMessagesWindow() {
        std::string title = "POCSAG Messages (" + name + ")###pager_msg_" + name;
        ImGui::SetNextWindowSize(ImVec2(700, 400), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(title.c_str(), &showMessagesWindow)) {
            ImGui::End();
            return;
        }

        // Prevent the waterfall from reacting to clicks/drags that originate
        // inside our window. Without this, dragging the title bar over the
        // waterfall area would move the selected VFO because the waterfall's
        // input handler uses raw mouse state plus a geometric hit test that
        // ignores overlapping ImGui windows. We use the public lock flag the
        // core resets to "showCredits" at the start of every MainWindow::draw,
        // so we only need to assert it when our window is actually engaged.
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows
                                   | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)
            || ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows))
        {
            gui::mainWindow.lockWaterfallControls = true;
        }

        // Toolbar
        if (ImGui::Button("Clear##pager_msg_clear")) {
            std::lock_guard<std::mutex> lck(messagesMtx);
            messages.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Save as TSV##pager_msg_save")) {
            saveAllMessagesToFile();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll##pager_msg_autoscroll", &autoScroll);
        ImGui::SameLine();
        if (ImGui::Checkbox("Hide errors##pager_msg_hideerr", &hideErrorMessages)) {
            saveSettings();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Hide tone-only##pager_msg_hidetone", &hideToneOnly)) {
            saveSettings();
        }

        // Messages table
        const ImGuiTableFlags tableFlags =
            ImGuiTableFlags_Borders     |
            ImGuiTableFlags_RowBg       |
            ImGuiTableFlags_Resizable   |
            ImGuiTableFlags_ScrollY     |
            ImGuiTableFlags_SizingStretchProp;

        if (ImGui::BeginTable(("##pager_msg_table_" + name).c_str(), 6, tableFlags)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Timestamp", ImGuiTableColumnFlags_WidthFixed, 150.0f);
            ImGui::TableSetupColumn("Address",   ImGuiTableColumnFlags_WidthFixed,  90.0f);
            ImGui::TableSetupColumn("Func",      ImGuiTableColumnFlags_WidthFixed,  40.0f);
            ImGui::TableSetupColumn("Type",      ImGuiTableColumnFlags_WidthFixed,  70.0f);
            ImGui::TableSetupColumn("FEC",       ImGuiTableColumnFlags_WidthFixed,  60.0f);
            ImGui::TableSetupColumn("Message",   ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            {
                std::lock_guard<std::mutex> lck(messagesMtx);
                for (const auto& m : messages) {
                    // Hide messages that still contain uncorrectable codewords
                    if (hideErrorMessages && m.errors > 0) { continue; }
                    // Hide tone-only (also catches noise-induced false-positive empties)
                    if (hideToneOnly && m.type == pocsag::MESSAGE_TYPE_TONE_ONLY) { continue; }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(formatTimestamp(m.timestamp).c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%u", (unsigned)m.address);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%u", (unsigned)m.function);

                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(typeName(m.type));

                    ImGui::TableSetColumnIndex(4);
                    if (m.errors > 0) {
                        // Only reachable when the user has disabled the filter
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                           "%d/%d", m.corrected, m.errors);
                    } else if (m.corrected > 0) {
                        ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f),
                                           "+%d", m.corrected);
                    } else {
                        ImGui::TextUnformatted("OK");
                    }

                    ImGui::TableSetColumnIndex(5);
                    ImGui::TextUnformatted(m.content.c_str());
                }
            }

            // Stick to the bottom when new messages arrive
            if (autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 10.0f) {
                ImGui::SetScrollHereY(1.0f);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

    void saveAllMessagesToFile() {
        // The "Save as TSV" button writes a one-shot SNAPSHOT of every
        // message currently in memory. This is distinct from "Log to file"
        // which appends in real time as each message arrives.
        // We name the snapshot with a timestamp so repeated saves don't
        // overwrite previous exports.
        std::time_t now = std::time(nullptr);
        char fnbuf[64];
        std::strftime(fnbuf, sizeof(fnbuf), "pocsag_%Y%m%d_%H%M%S.tsv",
                      std::localtime(&now));

        // Use the same folder the user picked for live logging if it's
        // valid, otherwise fall back to the SDR++ root.
        std::string dir;
        if (logFolderSelect.pathIsValid() && !logFolderSelect.path.empty()) {
            dir = logFolderSelect.expandString(logFolderSelect.path);
        } else {
            dir = (std::string)core::args["root"];
        }
        std::string outPath = dir + "/" + fnbuf;

        std::ofstream f(outPath);
        if (!f.is_open()) {
            flog::error("POCSAG: cannot open {} for writing", outPath);
            return;
        }
        f << "timestamp\taddress\tfunction\ttype\tmessage\n";
        std::lock_guard<std::mutex> lck(messagesMtx);
        int written = 0;
        for (const auto& m : messages) {
            // Apply the same filters as the GUI table so the export matches
            // what the user is currently seeing
            if (hideErrorMessages && m.errors > 0)                            { continue; }
            if (hideToneOnly       && m.type == pocsag::MESSAGE_TYPE_TONE_ONLY) { continue; }
            f << formatTimestamp(m.timestamp)
              << '\t' << m.address
              << '\t' << (int)m.function
              << '\t' << typeName(m.type)
              << '\t' << m.content
              << '\n';
            written++;
        }
        flog::info("POCSAG: saved {} messages to {}", written, outPath);
    }

    // -------- Members ------------------------------------------------
    std::string                       name;
    bool                              enabled = true;

    VFOManager::VFO*                  vfo = nullptr;
    std::unique_ptr<POCSAGDecoder>    decoder;

    // Messages buffer (DSP-thread producer / main-thread consumer)
    std::mutex                        messagesMtx;
    std::deque<pocsag::Message>       messages;

    // UI/runtime options
    bool                              showMessagesWindow = false;
    bool                              autoScroll         = true;
    bool                              hideErrorMessages  = true;  // Default: only show clean / corrected messages
    bool                              hideToneOnly       = true;  // Default: hide tone-only (suppresses most false-positive noise)
    bool                              logToFile          = false;
    FolderSelect                      logFolderSelect;            // Initialized in the constructor to "%ROOT%"

    // VFO snap interval (Hz). Default 1 kHz - common for handheld tuning
    // on POCSAG channels, where carriers sit on integer kHz boundaries.
    int                               snapId = 0;
    OptionList<int, int>              snapIntervals;
};

MOD_EXPORT void _INIT_() {
    // Initial default config
    json def = json({});
    std::string root = (std::string)core::args["root"];
    config.setPath(root + "/pager_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new PagerDecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (PagerDecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
