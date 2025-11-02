#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


void clear_screen() {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

// =============================
// Selection Functions
// =============================

int select_robot_type() {
    int choice = 0;
    printf("Select your type of Robot\n");
    printf("1. Inspection\n");
    printf("2. Delivery\n");

    while (choice != 1 && choice != 2) {
        printf("\nEnter choice (1 or 2): ");
        scanf("%d", &choice);
        if (choice != 1 && choice != 2)
            printf("Invalid input, please enter 1 or 2.\n");
    }
    return choice;
}

int select_mode() {
    int choice = 0;
    printf("\nSelect your Additional Mode:\n");
    printf("1. Low Battery Homing\n");
    printf("2. Home Detection\n");
    printf("3. None\n");

    while (choice < 1 || choice > 3) {
        printf("\nEnter choice (1, 2 or 3): ");
        scanf("%d", &choice);
        if (choice < 1 || choice > 3)
            printf("Invalid input, please enter 1, 2, or 3.\n");
    }
    return choice;
}

// =============================
// Main Entry
// =============================

int main() {
    clear_screen();
    printf("=== Warehouse Robot DevKit ===\n\n");

    int robot_choice = select_robot_type();
    int mode_choice  = select_mode();

    // Robot selection variables
    const char *robot_package = NULL;
    const char *robot_launch  = NULL;

    // Mode selection variables
    const char *mode_package_1 = NULL;
    const char *mode_launch_1  = NULL;
    const char *mode_package_2 = NULL;
    const char *mode_launch_2  = NULL;

    // =============================
    // Robot type mapping
    // =============================
    if (robot_choice == 1) { // Inspection robot
        robot_package = "inspection_bringup";
        robot_launch  = "inspection.launch.py"; // Launches wall_follower + aruco
    } else {
        robot_package = "delivery_bringup";
        robot_launch  = "delivery.launch.py"; // (leave empty for now)
    }

    // =============================
    // Mode mapping
    // =============================
    switch (mode_choice) {
        case 1: // Low Battery Homing
            mode_package_1 = "robot_drive";
            mode_launch_1  = "low_battery_homing.launch.py";
            mode_package_2 = "battery_monitor";
            mode_launch_2  = "battery_monitor.launch.py";
            break;
        case 2: // Home Detection
            mode_package_1 = "robot_drive";
            mode_launch_1  = "home_detection.launch.py";
            break;
        case 3: // None
        default:
            break;
    }

    // =============================
    // Summary
    // =============================
    printf("\nSummary of selections:\n");
    printf("Robot Type: %s\n", robot_choice == 1 ? "Inspection" : "Delivery");
    if (mode_choice == 1)
        printf("Mode: Low Battery Homing\n");
    else if (mode_choice == 2)
        printf("Mode: Home Detection\n");
    else
        printf("Mode: None\n");

    printf("\nPress ENTER to confirm and launch (or Ctrl+C to quit)...");
    getchar(); getchar();

// =============================
// Launch commands
// =============================
char command[256];

// --- Launch main robot type ---
snprintf(command, sizeof(command), "ros2 launch %s %s &", robot_package, robot_launch);
printf("\nLaunching: %s\n", command);
system(command);

// Small startup delay (optional)
sleep(2);

// --- Additional modes ---
if (mode_choice == 1) { // Low Battery Homing
    const char *parallel_launches[] = {
        "ros2 launch robot_drive low_battery_homing.launch.py &",
        "ros2 launch battery_monitor battery_monitor.launch.py &",
        "ros2 launch aruco_detector aruco_pose.launch.py &"
    };

    for (int i = 0; i < 3; i++) {
        printf("\nLaunching: %s\n", parallel_launches[i]);
        system(parallel_launches[i]);
        sleep(1);  // optional small delay between launches
    }
}

else if (mode_choice == 2) { // Home Detection mode
    const char *parallel_launches[] = {
        "ros2 launch robot_drive home_detection.launch.py &",
        "ros2 launch aruco_detector aruco_pose.launch.py &"
    };

    for (int i = 0; i < 2; i++) {
        printf("\nLaunching: %s\n", parallel_launches[i]);
        system(parallel_launches[i]);
        sleep(1);  // optional small delay
    }
}

printf("\nAll selected nodes launched successfully.\n");
printf("Press Ctrl+C to stop this program (ROS nodes will keep running in background).\n");

while (1) {
    sleep(1);
}

return 0 ;
}
