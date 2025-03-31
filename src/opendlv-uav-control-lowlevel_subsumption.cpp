#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cstdlib>

// 模擬感測器數值與 Crazyflie 控制指令的全域變數
// In a real implementation, these values would be updated via sensor callbacks.
std::atomic<float> sensorFront(0.5f);   // Front distance sensor reading
std::atomic<float> sensorLeft(0.5f);    // Left distance sensor reading
std::atomic<float> sensorRight(0.5f);   // Right distance sensor reading
std::atomic<float> sensorRear(0.5f);    // Rear distance sensor reading
std::atomic<float> batteryState(4.0f);  // Battery voltage
std::atomic<float> targetDistance(-1.0f);  // Target (green ball or charging pad) distance (-1 means not detected)
std::atomic<float> targetAngle(-4.0f);       // Target angle (-4 means not detected)

// Atomic flags for each behavior activation status
std::atomic<bool> dynamicObstacleActive(false); // Highest priority: dynamic obstacle avoidance
std::atomic<bool> staticObstacleActive(false);    // Static obstacle avoidance
std::atomic<bool> targetFindingActive(false);     // Target finding (turning toward target)
std::atomic<bool> frontReachingActive(false);       // Moving forward along a clear path
std::atomic<bool> lookAroundActive(false);          // Searching for a clear path (low priority)

std::mutex coutMutex; // Mutex to protect console output

// Dummy functions simulating the Crazyflie control commands.
// In the real system, these functions would send messages via OD4Session.
void Takeoff(float height, int duration) {
    std::lock_guard<std::mutex> lock(coutMutex);
    std::cout << "[Command] Taking off to height: " << height << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration * 1000));
}

void Goto(float x, float y, float z, float yaw, int duration) {
    std::lock_guard<std::mutex> lock(coutMutex);
    std::cout << "[Command] Goto position: x=" << x << ", y=" << y 
              << ", z=" << z << ", yaw=" << yaw << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration * 1000));
}

void Landing(float height, int duration) {
    std::lock_guard<std::mutex> lock(coutMutex);
    std::cout << "[Command] Landing to height: " << height << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration * 1000));
}

void Stopping() {
    std::lock_guard<std::mutex> lock(coutMutex);
    std::cout << "[Command] Stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

/*
    Behavior: Dynamic Obstacle Avoidance
    - This thread monitors sensor data for sudden obstacles (e.g., detected via vision or IR).
    - If an obstacle is detected too close in front, execute a dynamic dodge maneuver.
    - This behavior has the highest priority.
*/
void dynamicObstacleAvoidance() {
    while (true) {
        // Check condition: if front sensor reading is below a critical threshold (e.g., 0.3 meters)
        if (sensorFront.load() < 0.3f) {
            dynamicObstacleActive = true;
            {
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "[Dynamic Obstacle] Obstacle detected, executing dynamic avoidance maneuver." << std::endl;
            }
            // Execute a dynamic avoidance maneuver (e.g., dodge to the left)
            Goto(-0.2f, 0.2f, 0.0f, 0.0f, 1);
            // After maneuver, reset the flag
            dynamicObstacleActive = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            dynamicObstacleActive = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
    Behavior: Static Obstacle Avoidance
    - This thread monitors for walls or static obstacles via rangefinder values.
    - If any sensor (front, left, or right) is too close, execute a static obstacle avoidance maneuver.
    - This behavior is overridden by dynamic avoidance.
*/
void staticObstacleAvoidance() {
    while (true) {
        if (sensorFront.load() < 0.5f || sensorLeft.load() < 0.3f || sensorRight.load() < 0.3f) {
            staticObstacleActive = true;
            {
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "[Static Obstacle] Wall detected, executing static avoidance maneuver." << std::endl;
            }
            // Execute static avoidance (for example, stop movement)
            Goto(0.0f, 0.0f, 0.0f, 0.0f, 0); // Stop command
            staticObstacleActive = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            staticObstacleActive = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
    Behavior: Target Finding
    - This thread monitors for a valid target (e.g., green ball or charging pad) if detected.
    - If a target exists and the battery state is above the takeoff threshold,
      the Crazyflie will turn toward the target.
*/
void targetFinding() {
    while (true) {
        if (targetDistance.load() > 0 && batteryState.load() > 3.8f) {
            targetFindingActive = true;
            {
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "[Target Finding] Target detected, executing target finding behavior." << std::endl;
            }
            // Turn towards target (using targetAngle)
            Goto(0.0f, 0.0f, 0.0f, targetAngle.load(), 2);
            targetFindingActive = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            targetFindingActive = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
    Behavior: Front Reaching
    - When the path is clear (e.g., front sensor reading is high enough) and no higher priority behavior is active,
      the Crazyflie moves forward along its current heading.
*/
void frontReaching() {
    while (true) {
        if (sensorFront.load() > 0.7f && !dynamicObstacleActive.load() && !staticObstacleActive.load()) {
            frontReachingActive = true;
            {
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "[Front Reaching] Path is clear, executing forward movement." << std::endl;
            }
            // Move forward command
            Goto(0.5f, 0.0f, 0.0f, 0.0f, 2);
            frontReachingActive = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            frontReachingActive = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
    Behavior: Look Around
    - When no other behavior is active, the Crazyflie performs a look-around maneuver.
    - This behavior rotates the drone gradually (e.g., 360° over time) to search for a clear path or target.
*/
void lookAround() {
    while (true) {
        if (!dynamicObstacleActive.load() && !staticObstacleActive.load() &&
            !targetFindingActive.load() && !frontReachingActive.load()) {
            lookAroundActive = true;
            {
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "[Look Around] No active higher-priority behavior, executing look-around." << std::endl;
            }
            // Rotate slightly to search for a clear path/target
            Goto(0.0f, 0.0f, 0.0f, 0.2f, 2);
            lookAroundActive = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*
    Arbitrator
    - This thread continuously checks the flags of each behavior and,根據優先順序，
      保證高優先行為（例如動態障礙避讓）能夠中斷並覆蓋低優先行為。
    - 由於各行為函式皆獨立執行，本範例的仲裁器只用來延遲或避免衝突。
*/
void arbitrator() {
    while (true) {
        // If a high priority behavior is active, simply wait.
        if (dynamicObstacleActive.load() || staticObstacleActive.load() ||
            targetFindingActive.load() || frontReachingActive.load() ||
            lookAroundActive.load()) {
            // In a實際系統中，此處可調整調度策略
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main() {
    // 初始動作：起飛
    Takeoff(1.0f, 3);

    // 建立各行為的執行緒
    std::thread t_dynamic(dynamicObstacleAvoidance);
    std::thread t_static(staticObstacleAvoidance);
    std::thread t_target(targetFinding);
    std::thread t_front(frontReaching);
    std::thread t_look(lookAround);
    std::thread t_arb(arbitrator);

    // 模擬主迴圈：更新感測器數值（實際上會由回呼函式更新）
    for (int i = 0; i < 50; i++) {
        // 模擬前方距離隨機變化 (0.5 ~ 1.0 米)
        sensorFront = 0.5f + static_cast<float>(rand() % 100) / 200.0f;
        sensorLeft = 0.5f;
        sensorRight = 0.5f;
        sensorRear = 0.5f;
        // 每 10 次迴圈模擬一次目標出現
        if (i % 10 == 0) {
            targetDistance = 0.8f;
            targetAngle = 0.1f;
        } else {
            targetDistance = -1.0f;
            targetAngle = -4.0f;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 結束動作：降落與停止
    Landing(0.0f, 3);
    Stopping();

    // 由於各行為執行緒為無限迴圈，這裡用 detach 讓程式結束時自動回收 (實際應用中需適當管理執行緒退出)
    t_dynamic.detach();
    t_static.detach();
    t_target.detach();
    t_front.detach();
    t_look.detach();
    t_arb.detach();

    return 0;
}