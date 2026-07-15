#include <Arduino.h>
#include <math.h>
#include "Movement.h"

struct RobotState
{
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
};

struct Point{      
    float x = 0.0;
    float y = 0.0;
};

enum class NavigationState
{
    IDLE,
    
    RotateToMidpoint,
    MoveToMidpoint,

    StopAtMidpoint,
    RelocalizeAtMidpoint,

    RotateToTarget,
    MoveToTarget,

    Arrived,
    Obstacle
};

enum class MovementCommand
{
    None,
    Forward,
    Back,
    TurnLeft,
    TurnRight,
    Stop,
    Neutral
};

class NavigationController{
public:
    void start(const RobotState& start_pose, const Point& target_point, unsigned long now_ms);
    void update(const RobotState& current_pose, bool new_kens_measurement, double front_distance, bool front_distance_valid, unsigned long now_ms);
    
    NavigationState getState() const;
    bool arrived() const;
    bool obstacleDetected() const;
    bool isRelocalizing() const;
    double distanceToTarget(const RobotState& pose) const;
    void emergencyStop(unsigned long now_ms);
    void reset();
    void releaseWheels();

private:
    static constexpr double pi_ = 3.14159265358979323846;
    
    static double normalizeAngle(double angle);

    static double calculateDistance(const RobotState& pose, const Point& point);
    static double calculateTargetAngle(const RobotState& pose, const Point& point);

    void rotateToPoint(const RobotState& pose, const Point& point, NavigationState& state, unsigned long now_ms);
};