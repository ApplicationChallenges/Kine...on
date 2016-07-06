#ifndef __CHALLENGE_H__
#   define __CHALLENGE_H__

#   include <cassert>

#   include <vector>
#   include <unordered_map>
#   include <memory>
#   include <algorithm>
#   include <thread>
#   include <mutex>
#   include <random>
#   include <chrono>
#   include <cmath>

#   include <boost/signals2.hpp> // BOOST -> add path


namespace Challenge
{
    // Clock ------------------------------------------------------------------
    //
    using Clock = std::chrono::steady_clock;


    // TimePoint --------------------------------------------------------------
    //
    using TimePoint = Clock::time_point;


    // Point ------------------------------------------------------------------
    //
    // Represents a 2-D Coordinate in (SI-)unit meters
    //
    class Point
    {
    public:

        Point() noexcept :
            Point(0, 0)
        {
        }

        Point(double x, double y) noexcept :
            x(x),
            y(y)
        {
        }

        Point(const Point&) = default;
        Point& operator = (const Point&) = default;

        double X() const noexcept
        {
            return x;
        }

        double Y() const noexcept
        {
            return y;
        }

        static Point RandomPosition(const Point& maxPosition)
        {
            static std::default_random_engine generator(Clock::now().time_since_epoch().count());

            const auto x = std::uniform_real_distribution<double>(0, maxPosition.X())(generator);
            const auto y = std::uniform_real_distribution<double>(0, maxPosition.Y())(generator);

            return{ x, y };
        }

        static double Distance(const Point& one, const Point& two) noexcept
        {
            const auto dx = one.X() - two.X();
            const auto dy = one.Y() - two.Y();

            const auto distance = std::sqrt(dx * dx + dy * dy); // Pythagoras

            return distance;
        }

    private:

        double x;
        double y;
    };


    // Field ------------------------------------------------------------------
    //
    // Represents a playfield.
    //
    class Field final
    {
    public:

        explicit Field(const Point& maxPosition) :
            maxPosition(maxPosition)
        {
        }

        Field(const Field&) = delete;
        Field& operator = (const Field&) = delete;

        const Point& MaxPosition() const noexcept
        {
            return maxPosition;
        }

        Point Clip(const Point& position) const noexcept
        {
            auto x = position.X();
            auto y = position.Y();

            x = (x < 0) ? 0 : ((x > maxPosition.X()) ? maxPosition.X() : x);
            y = (y < 0) ? 0 : ((y > maxPosition.Y()) ? maxPosition.Y() : y);

            return{ x,y };
        }

        // TODO: add more methods on demand

    private:

        const Point maxPosition;
    };


    // ObjectData (POD) -------------------------------------------------------
    //
    struct ObjectData
    {
        ObjectData() = default;
        ObjectData(const ObjectData&) = default;
        ObjectData& operator = (const ObjectData&) = default;

        ObjectData(const TimePoint& when, const Point& where) :
            when(when),
            where(where),
            distance(0),
            speed(0)
        {
        }

        TimePoint when;
        Point where;
        double distance;
        double speed;
    };


    // Object -----------------------------------------------------------------
    //
    // Represents a (moving) object on a playfield.
    //
    class Object
    {
    public:

        Object(std::shared_ptr<Field> field, Point startPosition) :
            field(field),
            lock(),
            moveCount(0),
            data(Clock::now(), startPosition)
        {
            assert(nullptr != this->field);
        }

        virtual ~Object() = default;

        Object(const Object&) = delete;
        Object& operator = (const Object&) = delete;

        std::shared_ptr<Field> PlayField() const noexcept
        {
            return field;
        }

        Point Position() const noexcept
        {
            const std::lock_guard<std::mutex> guard{ lock };

            return data.where;
        }

        ObjectData GetData() const noexcept
        {
            const std::lock_guard<std::mutex> guard{ lock };

            return data;
        }

        Point Move()
        {
            const auto timeStamp(Clock::now());

            const std::lock_guard<std::mutex> guard{ lock };

            if (0 == moveCount)
            {
                data.when = timeStamp;

                //

                ++moveCount;
            }
            else
            {
                if (timeStamp > data.when)
                {
                    const auto duration = std::chrono::duration_cast<
                        std::chrono::duration<double>>(timeStamp - data.when).count();
                    assert(duration > 0);

                    //

                    const auto position = field->Clip(MoveTo(data, duration, moveCount));

                    //

                    const auto distance = Point::Distance(data.where, position);

                    data.when = timeStamp;
                    data.where = position;
                    data.distance += distance;
                    data.speed = distance / duration;

                    //

                    ++moveCount;
                }
            }

            return data.where;
        }

    protected:

        virtual Point MoveTo(const ObjectData& data, double duration, int moveCount) = 0;

    private:

        std::shared_ptr<Field> field;

        mutable std::mutex lock;

        int moveCount;

        ObjectData data;
    };


    // Human ------------------------------------------------------------------
    //
    // Represents a (moving) object on a playfield which behaves human like.
    //
    class Human : public Object
    {
    public:

        using Object::Object;

    protected:

        Point MoveTo(const ObjectData& data, double duration, int moveCount) override
        {
            assert(duration > 0);
            assert(moveCount > 0);
            
            // quick and dirty test...

            const auto pi = std::atan(1) * 4;

            static std::default_random_engine generator(Clock::now().time_since_epoch().count());

            const auto speed = std::uniform_real_distribution<double>(1, 5)(generator);
            const auto alpha = std::uniform_real_distribution<double>(0, 2 * pi)(generator);

            const auto distance = speed * duration;

            return{ data.where.X() + distance * std::cos(alpha), data.where.Y() + distance * std::sin(alpha) };
        }
    };


    // Sensor -----------------------------------------------------------------
    //
    class Sensor final
    {
    public:

        boost::signals2::signal<void(int who, TimePoint when, Point where)> signal;

        //

        // Constructs a Sensor with 
        //
        // the given ID, 
        // the given frequency (in Hz)
        // the specified max. noise (in (SI-unit) meters)
        // the associated target object
        //
        Sensor(int sensorID, double sensorFrequency, double sensorNoise, std::shared_ptr<Object> target) :
            sensorID(sensorID),
            sensorFrequency(sensorFrequency),
            sensorNoise((sensorNoise < 0) ? -sensorNoise : sensorNoise),
            target(target)
        {
            assert(nullptr != target);
        }

        Sensor(const Sensor&) = delete;
        Sensor& operator = (const Sensor&) = delete;

        int SensorID() const noexcept
        {
            return sensorID;
        }

        double SensorFrequency() const noexcept
        {
            return sensorFrequency;
        }

        double SensorNoise() const noexcept
        {
            return sensorNoise;
        }

        std::shared_ptr<Object> Target() /* const */
        {
            return target;
        }

        std::pair<TimePoint, Point> Measure() /* const */
        {
//          if (nullptr != target)
            {
                const auto timeStamp(Clock::now());
                const auto position(target->Position());

                const auto noisyPosition(AddRandomNoise(position));

                signal(sensorID, timeStamp, noisyPosition);

                return{ timeStamp, noisyPosition };
            }
        }

    private:

        Point AddRandomNoise(const Point& position)
        {
            static std::default_random_engine generator(
                Clock::now().time_since_epoch().count());

            std::uniform_real_distribution<double> distribution(-sensorNoise, sensorNoise);

            auto dx = distribution(generator);
            auto dy = distribution(generator);

            return{ position.X() + dx, position.Y() + dy };
        }

        const int sensorID;
        const double sensorFrequency;
        const double sensorNoise;

        std::shared_ptr<Object> target;
    };


    // PositionStreamProcessor ------------------------------------------------
    //
    class PositionStreamProcessor final
    {
    public:

        boost::signals2::signal<void(int who, double totalDistance, double currentSpeed)> signal;

        PositionStreamProcessor(std::shared_ptr<Field> field) :
            field(field)
        {
        }

        bool GetObjectData(int sensorID, ObjectData& data) const noexcept
        {
            const std::lock_guard<std::mutex> guard{ lock };

            const auto it = sensorData.find(sensorID);

            const auto available = (sensorData.end() != it);

            if (available)
            {
                assert(it->second != nullptr);

                data = *it->second;
            }

            return available;
        
        }

        void Collect(int who, TimePoint when, Point where)
        {
//            where = field->Clip(where);
            
            // TODO: clip position?

            const std::lock_guard<std::mutex> guard{ lock };

            auto it = sensorData.find(who);

            if (sensorData.end() == it)
            {
                sensorData[who] = std::make_unique<ObjectData>(when, where); // TODO: use emplace instead?
            }
            else
            {
                ObjectData& data = *it->second;

                //

                if (when > data.when) // throw away "expired" updates (should not happen during simulation)
                {
                    const auto duration = std::chrono::duration_cast<
                        std::chrono::duration<double>>(when - data.when).count();
                    assert(duration > 0);

                    const auto distance = Point::Distance(where, data.where);

                    //

                    data.when = when;
                    data.where = where;
                    data.distance += distance;
                    data.speed = distance / duration;

                    //

                    signal(who, data.distance, data.speed);
                }
            }
        }

    private:

        std::shared_ptr<Field> field;

        std::unordered_map<int, std::unique_ptr<ObjectData>> sensorData;
        mutable std::mutex lock;
    };
}

#endif
