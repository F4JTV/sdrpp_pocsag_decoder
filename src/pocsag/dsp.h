#pragma once
#include <dsp/stream.h>
#include <dsp/buffer/reshaper.h>
#include <dsp/sink/handler_sink.h>
#include <dsp/demod/quadrature.h>
#include <dsp/clock_recovery/mm.h>
#include <dsp/filter/fir.h>
#include <dsp/taps/tap.h>
#include <dsp/taps/from_array.h>
#include <dsp/digital/binary_slicer.h>
#include <vector>
#include <cmath>
#include <cassert>

// POCSAG nominal peak deviation. The original ITU-R recommendation is
// +/-4.5 kHz; the demod is given a negative value so the soft-decision sign
// follows the POCSAG bit convention (low freq = '1').
#define POCSAG_DEVIATION   4500.0

class POCSAGDSP : public dsp::Processor<dsp::complex_t, uint8_t> {
    using base_type = dsp::Processor<dsp::complex_t, uint8_t>;

public:
    POCSAGDSP() {}
    POCSAGDSP(dsp::stream<dsp::complex_t>* in, double samplerate, double baudrate) {
        init(in, samplerate, baudrate);
    }

    ~POCSAGDSP() {
        // The FIR releases its own internal buffer in its destructor; we
        // only own the tap storage we allocated.
        if (shape.taps) {
            dsp::taps::free(shape);
        }
    }

    void init(dsp::stream<dsp::complex_t>* in, double samplerate, double baudrate) {
        _samplerate = samplerate;
        _baudrate   = baudrate;

        // FM quadrature demodulator. Negative deviation gives the correct
        // POCSAG polarity straight out of the binary slicer.
        demod.init(NULL, -POCSAG_DEVIATION, _samplerate);

        // Build initial FIR matched filter (boxcar over one symbol period)
        shape = makeFirTaps(_samplerate, _baudrate);
        fir.init(NULL, shape);

        // Mueller & Muller clock recovery
        double sps = _samplerate / _baudrate;
        recov.init(NULL, sps, 1e-4, 1.0, 0.05);

        // We do not use the intermediate stream buffers directly
        fir.out.free();
        recov.out.free();

        base_type::init(in);
    }

    // Change the baudrate at runtime. Rebuilds the FIR matched filter and
    // updates the clock recovery loop omega. Safe to call while running.
    void setBaudrate(double baudrate) {
        assert(base_type::_block_init);
        std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
        base_type::tempStop();

        _baudrate = baudrate;
        double sps = _samplerate / _baudrate;

        // Allocate NEW taps first, swap them into the FIR, then free the
        // OLD ones. This keeps the FIR pointing at valid memory through the
        // whole transition even though tempStop() has already paused it.
        dsp::tap<float> oldShape = shape;
        shape = makeFirTaps(_samplerate, _baudrate);
        fir.setTaps(shape);
        if (oldShape.taps) { dsp::taps::free(oldShape); }

        // Update clock recovery omega for the new symbol rate
        recov.setOmega(sps);

        base_type::tempStart();
    }

    int process(int count, dsp::complex_t* in, float* softOut, uint8_t* out) {
        count = demod.process(count, in, demod.out.readBuf);
        count = fir.process(count, demod.out.readBuf, demod.out.readBuf);
        count = recov.process(count, demod.out.readBuf, softOut);
        dsp::digital::BinarySlicer::process(count, softOut, out);
        return count;
    }

    int run() {
        int count = base_type::_in->read();
        if (count < 0) { return -1; }

        count = process(count,
                        base_type::_in->readBuf,
                        soft.writeBuf,
                        base_type::out.writeBuf);

        base_type::_in->flush();
        if (!base_type::out.swap(count)) { return -1; }
        if (count) { if (!soft.swap(count)) { return -1; } }
        return count;
    }

    // Soft-decision stream used by the symbol diagram in the GUI
    dsp::stream<float> soft;

private:
    // Build a normalized boxcar (moving-average) FIR of length equal to
    // one symbol period. This acts as a simple matched filter for NRZ data.
    static dsp::tap<float> makeFirTaps(double samplerate, double baudrate) {
        int len = (int)std::round(samplerate / baudrate);
        if (len < 4)   { len = 4;   }
        if (len > 256) { len = 256; }

        std::vector<float> tmp((size_t)len, 1.0f / (float)len);
        return dsp::taps::fromArray<float>(len, tmp.data());
    }

    dsp::demod::Quadrature             demod;
    dsp::tap<float>                    shape{};
    dsp::filter::FIR<float, float>     fir;
    dsp::clock_recovery::MM<float>     recov;

    double _samplerate = 0.0;
    double _baudrate   = 0.0;
};
