#include "raylib.h"
#include "raymath.h"
#include "serial/serial.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

// Constants
const float LINK_RADIUS = 0.06f, JOINT_RADIUS = 0.08f;
const int PANEL_WIDTH = 320, MAX_INPUT_CHARS = 12;
const float LINK_LENGTH = 105.0f;  // Real arm link length in mm
const float SCALE = 0.01f;         // Scale factor for display (1mm = 0.01 units)

// State - Base at origin, two 105mm links
Vector3 joints[3] = {
    {0, 0, 0},                                    // Base joint at origin
    {0, LINK_LENGTH * SCALE, 0},                  // Shoulder joint (105mm up)
    {LINK_LENGTH * SCALE, LINK_LENGTH * SCALE, 0} // End effector (105mm forward)
};
bool draggingEnd = false;
serial::Serial* arduinoSerial = nullptr;
bool serialConnected = false;

// Serial
bool InitSerial(const char* port, unsigned long baud) {
    try {
        arduinoSerial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
        serialConnected = arduinoSerial->isOpen();
        printf(serialConnected ? "Serial connected\n" : "Serial failed\n");
        return serialConnected;
    } catch (...) {
        printf("Serial exception\n");
        return false;
    }
}

void SendToArduino(float x, float y, float z) {
    if (!serialConnected) return;
    char buf[64];
    snprintf(buf, 64, "%.1f %.1f %.1f\n", x, y, z);
    arduinoSerial->write(buf);
    printf("Sent: %s", buf);
}

void SendCommand(const char* cmd) {
    if (!serialConnected) return;
    arduinoSerial->write(std::string(cmd) + "\n");
    printf("Cmd: %s\n", cmd);
}

// Conversion
float simToMM(float sim) { return sim / SCALE; }
float mmToSim(float mm) { return mm * SCALE; }

// Drawing
void DrawLink(Vector3 a, Vector3 b, Color c) { DrawCylinderEx(a, b, LINK_RADIUS, LINK_RADIUS, 12, c); }
void DrawJoint(Vector3 p, Color c) { DrawSphere(p, JOINT_RADIUS, c); }

// Ray intersection
bool RayHitsSphere(Ray ray, Vector3 center, float radius) {
    Vector3 oc = Vector3Subtract(ray.position, center);
    float b = Vector3DotProduct(ray.direction, oc);
    float c = Vector3DotProduct(oc, oc) - radius * radius;
    return (b*b - c) >= 0;
}

bool RayHitsPlane(Ray ray, Vector3 planePoint, Vector3 normal, Vector3 *out) {
    float denom = Vector3DotProduct(ray.direction, normal);
    if (fabs(denom) < 1e-6f) return false;
    float t = Vector3DotProduct(Vector3Subtract(planePoint, ray.position), normal) / denom;
    if (t < 0) return false;
    *out = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
    return true;
}

void UpdateArmIK(float x, float y, float z) {
    // 2D IK in the XZ plane (horizontal reach) and Y (vertical)
    float reach = sqrt(x*x + z*z);  // Horizontal distance
    float dist = sqrt(reach*reach + y*y);  // Total distance to target
    
    // Check if reachable
    float maxReach = 2.0f * LINK_LENGTH;
    if (dist > maxReach) {
        // Clamp to max reach
        float scale = maxReach / dist;
        x *= scale;
        y *= scale;
        z *= scale;
        reach = sqrt(x*x + z*z);
        dist = maxReach;
    }
    
    // Law of cosines for 2-link IK
    float cosElbow = (LINK_LENGTH*LINK_LENGTH + LINK_LENGTH*LINK_LENGTH - dist*dist) / (2.0f * LINK_LENGTH * LINK_LENGTH);
    cosElbow = fmaxf(-1.0f, fminf(1.0f, cosElbow));
    float elbowAngle = acosf(cosElbow);
    
    float cosShoulder = (LINK_LENGTH*LINK_LENGTH + dist*dist - LINK_LENGTH*LINK_LENGTH) / (2.0f * LINK_LENGTH * dist);
    cosShoulder = fmaxf(-1.0f, fminf(1.0f, cosShoulder));
    float shoulderAngle = atan2f(y, reach) + acosf(cosShoulder);
    
    // Calculate joint positions in 3D
    float baseAngle = atan2f(x, z);  // Rotation around Y axis
    
    // Joint 1 (shoulder)
    joints[1].x = LINK_LENGTH * SCALE * sinf(baseAngle) * cosf(shoulderAngle);
    joints[1].y = LINK_LENGTH * SCALE * sinf(shoulderAngle);
    joints[1].z = LINK_LENGTH * SCALE * cosf(baseAngle) * cosf(shoulderAngle);
    
    // Joint 2 (end effector) - convert back from mm
    joints[2].x = x * SCALE;
    joints[2].y = y * SCALE;
    joints[2].z = z * SCALE;
}

// TextBox
struct TextBox {
    Rectangle bounds;
    char text[MAX_INPUT_CHARS];
    bool active, edited;
    const char* placeholder;
};

void UpdateTextBox(TextBox& box) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        box.active = CheckCollisionPointRec(GetMousePosition(), box.bounds);
    
    if (!box.active) return;
    SetMouseCursor(MOUSE_CURSOR_IBEAM);
    
    int key = GetCharPressed();
    while (key > 0) {
        if (key >= 32 && key <= 125) {
            if (!box.edited) { box.text[0] = '\0'; box.edited = true; }
            if (strlen(box.text) < MAX_INPUT_CHARS - 1) {
                int len = strlen(box.text);
                box.text[len] = (char)key;
                box.text[len + 1] = '\0';
            }
        }
        key = GetCharPressed();
    }
    
    if (IsKeyPressed(KEY_BACKSPACE) && strlen(box.text) > 0)
        box.text[strlen(box.text) - 1] = '\0';
}

void DrawTextBox(const TextBox& box) {
    DrawRectangleRec(box.bounds, LIGHTGRAY);
    DrawRectangleLinesEx(box.bounds, 1, box.active ? BLUE : DARKGRAY);
    DrawText(box.edited ? box.text : box.placeholder, (int)box.bounds.x + 6, (int)box.bounds.y + 8, 28, box.edited ? MAROON : GRAY);
}

// Main
int main() {
    const int W = 1320, H = 1000;
    InitWindow(W, H, "Robot Arm Dashboard");
    SetTargetFPS(60);
    InitSerial("COM4", 115200);

    Camera camera = {{3, 2, 3}, {1, 1, 0}, {0, 1, 0}, 45, CAMERA_PERSPECTIVE};
    
    TextBox boxes[3] = {
        {{(float)(W-PANEL_WIDTH+20), 200, (float)(PANEL_WIDTH-40), 40}, "", false, false, "X (mm)"},
        {{(float)(W-PANEL_WIDTH+20), 260, (float)(PANEL_WIDTH-40), 40}, "", false, false, "Y (mm)"},
        {{(float)(W-PANEL_WIDTH+20), 320, (float)(PANEL_WIDTH-40), 40}, "", false, false, "Z (mm)"}
    };
    
    Rectangle sendBtn = {(float)(W-PANEL_WIDTH+20), 380, (float)(PANEL_WIDTH-40), 50};
    Rectangle homeBtn = {(float)(W-PANEL_WIDTH+20), 445, (float)(PANEL_WIDTH-40), 50};

    while (!WindowShouldClose()) {
        // Update
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
            UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            draggingEnd = RayHitsSphere(GetScreenToWorldRay(GetMousePosition(), camera), joints[2], JOINT_RADIUS);
        
        if (draggingEnd && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            Vector3 hit, n = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
            if (RayHitsPlane(ray, joints[2], n, &hit)) {
                // Update arm with IK
                UpdateArmIK(simToMM(hit.x), simToMM(hit.y), simToMM(hit.z));
            }
        }
        
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) draggingEnd = false;
        
        if (draggingEnd) {
            snprintf(boxes[0].text, MAX_INPUT_CHARS, "%.1f", simToMM(joints[2].x));
            snprintf(boxes[1].text, MAX_INPUT_CHARS, "%.1f", simToMM(joints[2].y));
            snprintf(boxes[2].text, MAX_INPUT_CHARS, "%.1f", simToMM(joints[2].z));
            boxes[0].edited = boxes[1].edited = boxes[2].edited = true;
        }
        
        for (int i = 0; i < 3; i++) UpdateTextBox(boxes[i]);
        
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), sendBtn)) {
            float x = atof(boxes[0].text), y = atof(boxes[1].text), z = atof(boxes[2].text);
            UpdateArmIK(x, y, z);
            SendToArduino(x, y, z);
        }
        
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), homeBtn))
            SendCommand("home");

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Robot Arm Dashboard", 20, 20, 30, DARKBLUE);
        
        BeginMode3D(camera);
        DrawLink(joints[0], joints[1], BLUE);
        DrawLink(joints[1], joints[2], GREEN);
        for (int i = 0; i < 3; i++) DrawJoint(joints[i], (i == 2 && draggingEnd) ? ORANGE : DARKGRAY);
        DrawGrid(15, 1.0f);
        EndMode3D();
        
        int px = W - PANEL_WIDTH + 20;
        DrawRectangle(W-PANEL_WIDTH, 0, PANEL_WIDTH, H, (Color){245,246,248,255});
        DrawLine(W-PANEL_WIDTH, 0, W-PANEL_WIDTH, H, DARKGRAY);
        
        DrawText("STATUS", px, 60, 20, BLACK);
        DrawText(serialConnected ? "Connected" : "Disconnected", px, 85, 18, serialConnected ? DARKGREEN : RED);
        
        DrawText("CURRENT (mm)", px, 120, 20, BLACK);
        DrawText(TextFormat("X:%.1f Y:%.1f Z:%.1f", simToMM(joints[2].x), simToMM(joints[2].y), simToMM(joints[2].z)), px, 145, 18, DARKGRAY);
        
        DrawText("TARGET (mm)", px, 180, 20, BLACK);  
        for (int i = 0; i < 3; i++) DrawTextBox(boxes[i]);
        
        Color btnColor = serialConnected ? LIGHTGRAY : (Color){200,200,200,255};
        DrawRectangleRec(sendBtn, btnColor);
        DrawRectangleLinesEx(sendBtn, 1, DARKGRAY);
        DrawText("SEND TO ARM", (int)sendBtn.x + 55, (int)sendBtn.y + 15, 20, serialConnected ? BLACK : GRAY);
        
        DrawRectangleRec(homeBtn, btnColor);
        DrawRectangleLinesEx(homeBtn, 1, DARKGRAY);
        DrawText("HOME (90°)", (int)homeBtn.x + 65, (int)homeBtn.y + 15, 20, serialConnected ? BLACK : GRAY);
        
        DrawText("RMB: Orbit | LMB: Drag end", px, H-40, 16, DARKGRAY);
        
        EndDrawing();
    }

    if (arduinoSerial) { arduinoSerial->close(); delete arduinoSerial; }
    CloseWindow();
    return 0;
}