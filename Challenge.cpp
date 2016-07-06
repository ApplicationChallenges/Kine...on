#include <cassert>

#include <vector>
#include <memory>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>

#include <boost/signals2.hpp>

#include "Challenge.h"


using namespace Challenge;

// PositionGenerator ------------------------------------------------------
//
class PositionGenerator final
{
    // We want an even objectCount...
    const int halfObjectCount = 10;
    const int objectCount = halfObjectCount << 1;

    const double sensorFrequency = 20;
    const double sensorNoise = 0.3;

public:

    boost::signals2::signal<void(int who, TimePoint when, Point where)> signal;

    void Generate(std::shared_ptr<Field> field)
    {
        assert(field != nullptr);

        //

        // Create some real-world multithreaded environment with 
        // 2x10 objects and 2x10 sensors.

        auto forwardSignal = [this](int who, TimePoint when, Point where)
        {
            this->signal(who, when, where);
        };

        std::vector<std::shared_ptr<Object>> objects1;
        std::vector<std::shared_ptr<Object>> objects2;

        objects1.reserve(halfObjectCount);
        objects2.reserve(halfObjectCount);
                
        std::vector<std::shared_ptr<Sensor>> sensors1;
        std::vector<std::shared_ptr<Sensor>> sensors2;

        sensors1.reserve(halfObjectCount);
        sensors2.reserve(halfObjectCount);

        for (int index = 0; index != halfObjectCount; ++index)
        {
            auto object1 = std::make_shared<Human>(
                field, Point::RandomPosition(field->MaxPosition()));

            const int sensor1ID = index;
            auto sensor1 = std::make_shared<Sensor>(
                sensor1ID, sensorFrequency, sensorNoise, object1);

            sensor1->signal.connect(forwardSignal);
            
            objects1.push_back(object1);
            sensors1.push_back(sensor1);

            //

            auto object2 = std::make_shared<Human>(
                field, Point::RandomPosition(field->MaxPosition()));

            const int sensor2ID = index + halfObjectCount;
            auto sensor2 = std::make_shared<Sensor>(
                sensor2ID, sensorFrequency, sensorNoise, object2);

            sensor2->signal.connect(forwardSignal);

            objects2.push_back(object2);
            sensors2.push_back(sensor2);
        }

        //

        std::thread thread1(&PositionGenerator::ThreadFunc, this, sensors1, objects2);
        std::thread thread2(&PositionGenerator::ThreadFunc, this, sensors2, objects1);

        thread1.join();
        thread2.join();
    }

private:

    void ThreadFunc(
        std::vector<std::shared_ptr<Sensor>>& sensors,
        std::vector<std::shared_ptr<Object>>& objects)
    {
        assert(!sensors.empty());
        assert(!objects.empty());
        assert(sensors.size() == objects.size());

        const int objectFrequencyFactor = 2;
        const double objectFrequency = sensorFrequency * objectFrequencyFactor;

        const auto interval = std::chrono::microseconds(
            (int)((1'000'000. / objectFrequency) / objects.size()));

        const auto sensorsBegin = sensors.begin();
        const auto sensorsEnd = sensors.end();

        const auto objectsBegin = objects.begin();
        const auto objectsEnd = objects.end();

        auto itSensors = sensorsBegin;
        auto itObjects = objectsBegin;

        int index = 0;

        for (;;)
        {
            (*itObjects)->Move();

            if (++itObjects == objectsEnd)
            {
                itObjects = objectsBegin;
            }

            if (++index == objectFrequencyFactor)
            {
                index = 0;

                (*itSensors)->Measure();

                if (++itSensors == sensorsEnd)
                {
                    itSensors = sensorsBegin;
                }
            }

            //

            std::this_thread::sleep_for(interval);
        }
    }
};


int
main()
{
    auto field = std::make_shared<Field>(Point(100, 100));

    PositionStreamProcessor psp(field);
    PositionGenerator pg;

    pg.signal.connect(
        boost::bind(&PositionStreamProcessor::Collect, &psp, _1, _2, _3));

    psp.signal.connect([](int who, double distance, double speed)
    {
        std::cout << who << " " << distance << " " << speed << std::endl;
    });

    pg.Generate(field);
}
