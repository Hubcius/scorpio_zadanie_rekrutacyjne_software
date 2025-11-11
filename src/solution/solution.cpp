#include "tester.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <fstream>
#include <mutex>
#include <condition_variable>


//std::ofstream logFile("/home/hubert/worskspace/pwr/scorpio_zadanie_rekrutacyjne_software/src/solution/motor_log.txt",std::ios::out | std::ios::trunc);

class Motor
{
public:
    enum class Direction
    {
        Up,
        Down
    };

    static std::pair<uint16_t, uint16_t> calculateMotorEndingPosition(const Point& point, Motor& motor1, Motor& motor2){
        
        std::pair<int, int> firstCaseAngle{
            (kFullCircleDegreesInt - calculateMotorAngle(point.x, point.y)) % kFullCircleDegreesInt,
            calculateMotorAngle(std::sqrt((point.x * point.x) + (point.y * point.y)), point.z)
        };
        std::pair<uint16_t, uint16_t> firstCasePosition{
            static_cast<uint16_t>(std::round(firstCaseAngle.first * (static_cast<double>(kEncoderReadingRange) / kFullCircleDegreesDouble))),
            static_cast<uint16_t>(std::round(firstCaseAngle.second * (static_cast<double>(kEncoderReadingRange) / kFullCircleDegreesDouble)))
        };
        std::pair<uint16_t, uint16_t> firstCaseDistance{
            motor1.shortestDinstanceChecking(firstCasePosition.first),
            motor2.shortestDinstanceChecking(firstCasePosition.second)
        };


        std::pair<int, int> secondCaseAngle{
            (firstCaseAngle.first + kHalfCircleDegreesInt) % kFullCircleDegreesInt,
            (firstCaseAngle.second < kHalfCircleDegreesInt) ? kHalfCircleDegreesInt - firstCaseAngle.second : kFullCircleDegreesInt - (firstCaseAngle.second - kHalfCircleDegreesInt)
        };
        secondCaseAngle.second = (secondCaseAngle.second < 0) ? (secondCaseAngle.second % kHalfCircleDegreesInt) + kHalfCircleDegreesInt : secondCaseAngle.second;
        std::pair<uint16_t, uint16_t> secondCasePosition{
            static_cast<uint16_t>(std::round(secondCaseAngle.first * (static_cast<double>(kEncoderReadingRange) / kFullCircleDegreesDouble))),
            static_cast<uint16_t>(std::round(secondCaseAngle.second * (static_cast<double>(kEncoderReadingRange) / kFullCircleDegreesDouble)))
        };
        std::pair<uint16_t, uint16_t> secondCaseDistance{
            motor1.shortestDinstanceChecking(secondCasePosition.first),
            motor2.shortestDinstanceChecking(secondCasePosition.second)
        };

        return (std::max(firstCaseDistance.first, firstCaseDistance.second) < std::max(secondCaseDistance.first, secondCaseDistance.second))? firstCasePosition :secondCasePosition;
        
    }

    void updatePosition(uint16_t newPosition){
        {
            std::lock_guard<std::mutex> lock(mutex_);
            previousPosition_ = currentPosition_;
            currentPosition_ = newPosition;
            if (std::abs(static_cast<int>(currentPosition_) - static_cast<int>(previousPosition_)) > kEncoderWrapThreshold){
                goingUpOrDown();
            }
        }

        std::cout << static_cast<int>(currentPosition_) << "\n";
        /*
        if(!idle_)
            logFile << static_cast<int>(currentPosition_) << std::endl;
        */

        

    }

    void speedPIDController(){
        std::lock_guard<std::mutex> lock(mutex_);
        double motorError{0};
        if(goOverOneCycle_){
            if(directionOfSpeed_ == Direction::Up)
                motorError = (static_cast<double>(targetPosition_) + static_cast<double>(kEncoderReadingRange)) - static_cast<double>(currentPosition_);
            else
                motorError = -1.0 * (static_cast<double>(kEncoderReadingRange) - static_cast<double>(targetPosition_)) - static_cast<double>(currentPosition_);
        }
        else{
            motorError = static_cast<double>(targetPosition_) - static_cast<double>(currentPosition_);
        }

        integral_ = std::clamp(integral_ + motorError * kdt, -kIntegralLimit, kIntegralLimit);


        double derivative{(motorError - previousMotorError_) / kdt};
        double output = kP * motorError + kI * integral_ + kD * derivative;
        previousMotorError_ = motorError;

        int output_int{static_cast<int>(std::round(output))};
        if(output_int < 0){
            output_int += -1 * kSmallestMotorValue;
            output_int += kMotorDeadzone;
        }
        else{
            output_int += kSmallestMotorValue;
            output_int -= kMotorDeadzone;
        }
        output_int = std::clamp(output_int, kMotorMinSpeed, kMotorMaxSpeed);
        speed_ = static_cast<int8_t>(output_int);
    }

    void motorStop() {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_ = 0;
        idle_ = true;
    }

    void setTargetPosition( uint16_t targetPosition){
        std::lock_guard<std::mutex> lock(mutex_);
        integral_ = 0.0;
        previousMotorError_ = 0.0;
        targetPosition_ = targetPosition;
        idle_ = false;
        goingUpOrDown();
    }

    int8_t getSpeed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return speed_; 
    }
    uint16_t getCurrentPosition() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return currentPosition_; 
    }

    bool isAtTarget() const { 
        std::lock_guard<std::mutex> lock(mutex_);
        return std::min(std::abs(static_cast<int>(currentPosition_) - static_cast<int>(targetPosition_)), static_cast<int>(kEncoderReadingRange) - std::abs(static_cast<int>(currentPosition_) - static_cast<int>(targetPosition_))) <= kMarginOfMotorTarget;
    }
    bool isIdle() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return idle_;
    }

private:
    static constexpr uint16_t kEncoderReadingRange{4095};
    static constexpr uint16_t kEncoderWrapThreshold{3000};
    static constexpr double kFullCircleDegreesDouble{360.0};
    static constexpr int kFullCircleDegreesInt{360};
    static constexpr double kHalfCircleDegreesDouble{180.0};
    static constexpr int kHalfCircleDegreesInt{180};
    static constexpr int kMotorDeadzone{3};
    static constexpr int kMotorMaxSpeed{127};
    static constexpr int kMotorMinSpeed{-128};
    static constexpr int kMarginOfMotorTarget{1};
    static constexpr double kP{3.0};
    static constexpr double kI{0.05};
    static constexpr double kD{0.1};
    static constexpr double kdt{0.05};
    static constexpr double kIntegralLimit{100.0};
    static constexpr double kSmallestMotorValue{58.0};

    mutable std::mutex mutex_;
    uint16_t currentPosition_{0};
    uint16_t previousPosition_{0};
    uint16_t targetPosition_{0};
    int8_t speed_{0};
    Direction directionOfSpeed_{Direction::Up};
    bool idle_{true};
    bool goOverOneCycle_{false};
    double integral_{0.0};
    double previousMotorError_{0.0};

    uint16_t shortestDinstanceChecking (uint16_t targetPosition) const {
        if (targetPosition > currentPosition_){
            return std::min(targetPosition - currentPosition_, currentPosition_ + (kEncoderReadingRange - targetPosition));
        }
        else{
            return std::min (currentPosition_ - targetPosition, (targetPosition + kEncoderReadingRange) - currentPosition_);
        }
    }

    void goingUpOrDown(){
        if (targetPosition_ > currentPosition_){
            if (targetPosition_ - currentPosition_ < currentPosition_ + (kEncoderReadingRange - targetPosition_)){
                directionOfSpeed_ = Direction::Up;
                goOverOneCycle_ = false;
            }
            else{
                directionOfSpeed_ = Direction::Down;
                goOverOneCycle_ = true;
            }
        }
        else{
            if (currentPosition_ - targetPosition_ < (targetPosition_ + kEncoderReadingRange) - currentPosition_){
                directionOfSpeed_ = Direction::Down;
                goOverOneCycle_ = false;
            }
            else{
                directionOfSpeed_ = Direction::Up;
                goOverOneCycle_ = true;
            }
        }
    }

    static int calculateMotorAngle(const double adjacent, const double opposite){
        double angleDegrees {atan2(opposite, adjacent) * kHalfCircleDegreesDouble / M_PI};
        if (angleDegrees < 0)
            angleDegrees += kFullCircleDegreesDouble;
        return static_cast<int>(std::round(angleDegrees));
    }
};




int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt)
{
    std::mutex m;
    std::condition_variable conditionVariable;

    Motor motor1;
    Motor motor2;
    auto commands = tester->get_commands();
    auto motor1Interface = tester->get_motor_1();
    auto motor2Interface = tester->get_motor_2();
    bool motorsAtTarget = false;

    double kStopDelay{10.1};
    auto pointInTime{std::chrono::steady_clock::now()};

    auto setTarget = [&](const Point &point) {
        auto [motor1TargetPosition, motor2TargetPosition] = Motor::calculateMotorEndingPosition(point, motor1, motor2);
        motor1.setTargetPosition(motor1TargetPosition);
        motor2.setTargetPosition(motor2TargetPosition);
        //logFile << motor1TargetPosition <<" "<< motor2TargetPosition << std::endl;

        motor1.speedPIDController();
        motor2.speedPIDController();

        motor1Interface->send_data(motor1.getSpeed());
        motor2Interface->send_data(motor2.getSpeed());
    };
    
    auto checkAndNotify = [&]() {
        std::lock_guard<std::mutex> lock(m);
        if (motor1.isAtTarget() && motor2.isAtTarget()) {
            auto now{std::chrono::steady_clock::now()};
            if(std::chrono::duration<double>{now - pointInTime}.count() > kStopDelay){
                motorsAtTarget = true;
                conditionVariable.notify_one();
                return true;
            }
            return false;
        }
        return false;
    };

    commands->add_data_callback([&](const Point &point){
        pointInTime = std::chrono::steady_clock::now();
        std::cout << "Command point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
        if(preempt){
            setTarget(point); 
        }
        else{
            kStopDelay = 0;
            setTarget(point); //this else is added, so the task "Dojazd do pojedyńczego celu" is still working perfectly
        }
    });

    motor1Interface->add_data_callback([&](const uint16_t &data){
        std::cout<<"Motor 1 ";
        motor1.updatePosition(data);
        if(!motor1.isIdle()){
            if(!checkAndNotify()){
                motor1.speedPIDController();
                motor1Interface->send_data(motor1.getSpeed());
            }
        }
        
    });

    motor2Interface->add_data_callback([&](const uint16_t &data){
        std::cout<<"Motor 2 ";
        motor2.updatePosition(data);
        if(!motor2.isIdle()){
            if(!checkAndNotify()){
                motor2.speedPIDController();
                motor2Interface->send_data(motor2.getSpeed());
            }
        }

    });

    {
        std::unique_lock<std::mutex> lock(m);
        conditionVariable.wait(lock, [&]{ return motorsAtTarget; });
    }

    motor1.motorStop();
    motor2.motorStop();
    motor1Interface->send_data(motor1.getSpeed());
    motor2Interface->send_data(motor2.getSpeed());

    //logFile.close();
    return 0;
}