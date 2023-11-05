#include "threaded_cout.hpp"

#include <debugapi.h>

#define protected public
#include "../hardware/decoder/decoder.hpp"
#include "../vehicle/rail/poweredrailvehicle.hpp"
#include "../vehicle/rail/railvehiclelist.hpp"
#include "../core/objectproperty.tpp"
#include "../train/train.hpp"
#include "../world/world.hpp"

extern std::mutex &get_cout_mutex()
{
    static std::mutex m;
    return m;
}

void real_print(const std::string &str)
{
    OutputDebugStringA(str.c_str());
}

void printInfo(const std::string& caption, const std::shared_ptr<const Decoder> &decoder)
{
    auto& world = decoder->world();
    const PoweredRailVehicle *vehicle = nullptr;
    for(const auto& v : *world.railVehicles.value().get())
    {
        if(auto p = std::dynamic_pointer_cast<PoweredRailVehicle>(v))
        {
            if(p->decoder.value() == decoder)
            {
                vehicle = p.get();
                break;
            }
        }
    }

    Train *train = vehicle ? vehicle->activeTrain.value().get() : nullptr;

    print(caption, "\n",
          "decoder: ", decoder->name.value(), "\n",
          "throttle: ", decoder->throttle, "\n",
          "step: ", (int)Decoder::throttleToSpeedStep(decoder->throttle, 126), "\n",
          "lastTrainThorttle: ", vehicle ? vehicle->lastTrainSpeedStep : -1, "\n",
          "lastTrainStep: ", vehicle ? (int)Decoder::throttleToSpeedStep(vehicle->lastTrainSpeedStep, 126) : -1, "\n",
          "trainCurSpeed: ", train ? train->speed.value() : -1, "\n",
          "trainTargetSpeed: ", train ? train->throttleSpeed.value() : -1, "\n",
          "trainMaxSpeed: ", train ? train->speedMax.value() : -1, "\n",
          "-------END-------\n\n"
    );
}
