#include "raylib.h"
#include <cmath>
#include <algorithm>

// =====================
// "C style" helpers
// =====================
struct Vec3 { float x,y,z; };
static inline Vec3 v3(float x,float y,float z){ return {x,y,z}; }
static inline Vec3 add(Vec3 a, Vec3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline Vec3 sub(Vec3 a, Vec3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }

static inline float clampf(float v, float lo, float hi){
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline Vec3 rotX(Vec3 p, float a){
    float c = cosf(a), s = sinf(a);
    return { p.x, c*p.y - s*p.z, s*p.y + c*p.z };
}
static inline Vec3 rotZ(Vec3 p, float a){
    float c = cosf(a), s = sinf(a);
    return { c*p.x - s*p.y, s*p.x + c*p.y, p.z };
}

// =====================
// Robot params (mm)
// =====================
static constexpr float A = 76.0f;  // femur
static constexpr float B = 99.0f;  // tibia
static constexpr float L = 52.0f;  // coxa

// Body geometry (mm)
static constexpr float bodyHalfWidth = 60.0f;
static constexpr float hipX = 0.0f;

// Neutral foot targets in BODY frame (mm)
static Vec3 footR0_body = { +40.0f, +120.0f, -90.0f };
static Vec3 footL0_body = { +40.0f, -120.0f, -90.0f };

// Hip mounts in BODY frame
static Vec3 hipR_body = { hipX, +bodyHalfWidth, 0.0f };
static Vec3 hipL_body = { hipX, -bodyHalfWidth, 0.0f };

// Middle leg mount yaw assumptions (adjust for your hardware)
static constexpr float mountYawR = -PI/2.0f;
static constexpr float mountYawL = +PI/2.0f;

// =====================
// IK (matches your Arduino math)
// inputs are in LEG frame
// =====================
static void IK(Vec3 p_leg, float &theta1, float &theta2, float &theta3)
{
    float x = p_leg.x, y = p_leg.y, z = p_leg.z;

    float R = sqrtf(x*x + y*y);
    theta1 = atan2f(y, x);

    float D = R - L;
    float C = sqrtf(D*D + (-z)*(-z));

    // protect acos
    float cos_b = (A*A + C*C - B*B) / (2.0f*A*C);
    cos_b = clampf(cos_b, -1.0f, 1.0f);
    float b = (C > 1e-6f) ? acosf(cos_b) : 0.0f;

    float g = atan2f(D, -z);
    theta2 = b + g;

    float cos_h = (A*A + B*B - C*C) / (2.0f*A*B);
    cos_h = clampf(cos_h, -1.0f, 1.0f);
    float h = acosf(cos_h);

    theta3 = PI - h; // same as 180-h in degrees
}

// =====================
// FK for visualization (simple convention)
// =====================
static void FK(float th1, float th2, float th3, Vec3 &hip, Vec3 &coxa, Vec3 &knee, Vec3 &foot)
{
    // in yaw-less plane (radial = +X, up = +Z, but our z-up and feet negative is fine)
    Vec3 hip0  = v3(0,0,0);
    Vec3 coxa0 = v3(L,0,0);

    Vec3 knee0 = add(coxa0, v3(A*sinf(th2), 0.0f, -A*cosf(th2)));

    float tibAbs = th2 - th3;
    Vec3 foot0 = add(knee0, v3(B*sinf(tibAbs), 0.0f, -B*cosf(tibAbs)));

    // yaw about Z
    hip  = rotZ(hip0,  th1);
    coxa = rotZ(coxa0, th1);
    knee = rotZ(knee0, th1);
    foot = rotZ(foot0, th1);
}

// Leg frame -> Body frame: bodyPoint = hipMount + Rz(mountYaw)*legPoint
static Vec3 LegToBody(Vec3 p_leg, Vec3 hipMount_body, float mountYaw){
    return add(hipMount_body, rotZ(p_leg, mountYaw));
}

// =====================
// Small 2D drawing helpers
// =====================
static Vector2 ProjectTop(Vec3 p, Vector2 origin, float s){
    // Top view: X-Y
    return { origin.x + p.x*s, origin.y - p.y*s };
}
static Vector2 ProjectFront(Vec3 p, Vector2 origin, float s){
    // Front view: Y-Z
    return { origin.x + p.y*s, origin.y - p.z*s };
}
static void DrawThickLine(Vector2 a, Vector2 b, float thickness, Color c){
    DrawLineEx(a, b, thickness, c);
}

// =====================
// Main
// =====================
int main()
{
    const int W = 1400, H = 520;
    InitWindow(W, H, "Hexapod Virtual Roll (2 legs) - raylib");
    SetTargetFPS(60);

    // Simple 3D camera
    Camera3D cam = {0};
    cam.position = (Vector3){ 250, 250, 180 };
    cam.target   = (Vector3){ 40, 0, -60 };
    cam.up       = (Vector3){ 0, 0, 1 };
    cam.fovy     = 45.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    float rollDeg = 0.0f;

    while (!WindowShouldClose())
    {
        // --- slider UI (simple)
        Rectangle slider = { 200, (float)H - 40, 900, 10 };
        DrawRectangleRec(slider, DARKGRAY);
        DrawRectangleLinesEx(slider, 1, GRAY);

        // map rollDeg [-20, 20] to slider x
        float t = (rollDeg + 20.0f) / 40.0f;
        float knobX = slider.x + t * slider.width;
        Rectangle knob = { knobX - 6, slider.y - 6, 12, slider.height + 12 };

        // mouse drag
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
        {
            Vector2 m = GetMousePosition();
            if (CheckCollisionPointRec(m, (Rectangle){slider.x, slider.y-20, slider.width, slider.height+40}))
            {
                float tt = (m.x - slider.x) / slider.width;
                tt = clampf(tt, 0.0f, 1.0f);
                rollDeg = tt*40.0f - 20.0f;
            }
        }

        // --- compute virtual roll
        float roll = rollDeg * (PI/180.0f);

        // inverse roll compensation in BODY frame: p_new = Rx^T * p0
        // For rot matrices, transpose = inverse; for X rotation, inverse is just -roll.
        Vec3 footR_body = rotX(footR0_body, -roll);
        Vec3 footL_body = rotX(footL0_body, -roll);

        // body->hip local
        Vec3 pR_hip = sub(footR_body, hipR_body);
        Vec3 pL_hip = sub(footL_body, hipL_body);

        // hip local BODY -> LEG
        Vec3 pR_leg = rotZ(pR_hip, -mountYawR);
        Vec3 pL_leg = rotZ(pL_hip, -mountYawL);

        // IK
        float th1R, th2R, th3R;
        float th1L, th2L, th3L;
        IK(pR_leg, th1R, th2R, th3R);
        IK(pL_leg, th1L, th2L, th3L);

        // FK (leg frame)
        Vec3 hipR_leg, coxaR_leg, kneeR_leg, footR_leg;
        Vec3 hipL_leg, coxaL_leg, kneeL_leg, footL_leg;
        FK(th1R, th2R, th3R, hipR_leg, coxaR_leg, kneeR_leg, footR_leg);
        FK(th1L, th2L, th3L, hipL_leg, coxaL_leg, kneeL_leg, footL_leg);

        // to BODY frame for drawing
        Vec3 hipR = LegToBody(hipR_leg,  hipR_body, mountYawR);
        Vec3 coxaR= LegToBody(coxaR_leg, hipR_body, mountYawR);
        Vec3 kneeR= LegToBody(kneeR_leg, hipR_body, mountYawR);
        Vec3 footR= LegToBody(footR_leg, hipR_body, mountYawR);

        Vec3 hipL = LegToBody(hipL_leg,  hipL_body, mountYawL);
        Vec3 coxaL= LegToBody(coxaL_leg, hipL_body, mountYawL);
        Vec3 kneeL= LegToBody(kneeL_leg, hipL_body, mountYawL);
        Vec3 footL= LegToBody(footL_leg, hipL_body, mountYawL);

        // --- draw
        BeginDrawing();
        ClearBackground((Color){20,20,20,255});

        DrawText("Top view (X-Y)", 30, 15, 20, RAYWHITE);
        DrawText("Front view (Y-Z)", 480, 15, 20, RAYWHITE);
        DrawText("3D view", 930, 15, 20, RAYWHITE);

        DrawText(TextFormat("Roll: %.1f deg", rollDeg), 30, H-55, 20, RAYWHITE);
        DrawRectangleRec(knob, RAYWHITE);

        // 2D view origins & scale
        Vector2 topOrigin   = { 230, 230 };
        Vector2 frontOrigin = { 680, 230 };
        float s = 1.2f;               // pixels per mm for 2D
        float thick = 5.0f;

        // Body line between hip mounts
        DrawThickLine(ProjectTop(hipL_body, topOrigin, s), ProjectTop(hipR_body, topOrigin, s), thick, GRAY);
        DrawThickLine(ProjectFront(hipL_body, frontOrigin, s), ProjectFront(hipR_body, frontOrigin, s), thick, GRAY);

        // Right leg (orange-ish)
        Color CR = ORANGE;
        DrawThickLine(ProjectTop(hipR, topOrigin, s),  ProjectTop(coxaR, topOrigin, s), thick, CR);
        DrawThickLine(ProjectTop(coxaR, topOrigin, s), ProjectTop(kneeR, topOrigin, s), thick, CR);
        DrawThickLine(ProjectTop(kneeR, topOrigin, s), ProjectTop(footR, topOrigin, s), thick, CR);

        DrawThickLine(ProjectFront(hipR, frontOrigin, s),  ProjectFront(coxaR, frontOrigin, s), thick, CR);
        DrawThickLine(ProjectFront(coxaR, frontOrigin, s), ProjectFront(kneeR, frontOrigin, s), thick, CR);
        DrawThickLine(ProjectFront(kneeR, frontOrigin, s), ProjectFront(footR, frontOrigin, s), thick, CR);

        // Left leg (green-ish)
        Color CL = LIME;
        DrawThickLine(ProjectTop(hipL, topOrigin, s),  ProjectTop(coxaL, topOrigin, s), thick, CL);
        DrawThickLine(ProjectTop(coxaL, topOrigin, s), ProjectTop(kneeL, topOrigin, s), thick, CL);
        DrawThickLine(ProjectTop(kneeL, topOrigin, s), ProjectTop(footL, topOrigin, s), thick, CL);

        DrawThickLine(ProjectFront(hipL, frontOrigin, s),  ProjectFront(coxaL, frontOrigin, s), thick, CL);
        DrawThickLine(ProjectFront(coxaL, frontOrigin, s), ProjectFront(kneeL, frontOrigin, s), thick, CL);
        DrawThickLine(ProjectFront(kneeL, frontOrigin, s), ProjectFront(footL, frontOrigin, s), thick, CL);

        // 3D
        BeginMode3D(cam);
        DrawGrid(10, 20.0f);

        // draw body bar
        DrawLine3D((Vector3){hipL_body.x, hipL_body.y, hipL_body.z},
                   (Vector3){hipR_body.x, hipR_body.y, hipR_body.z}, GRAY);

        auto dl3 = [](Vec3 a, Vec3 b, Color c){
            DrawLine3D((Vector3){a.x,a.y,a.z}, (Vector3){b.x,b.y,b.z}, c);
        };

        // right
        dl3(hipR, coxaR, ORANGE);
        dl3(coxaR, kneeR, ORANGE);
        dl3(kneeR, footR, ORANGE);

        // left
        dl3(hipL, coxaL, LIME);
        dl3(coxaL, kneeL, LIME);
        dl3(kneeL, footL, LIME);

        EndMode3D();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
