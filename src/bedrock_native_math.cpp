#include <napi.h>
#include <cmath>
#include <cstdint>
#include <emmintrin.h>

namespace {

constexpr float DEG_TO_RAD         = 0.01745329238474369f;
constexpr float BDS_EPSILON        = 1.1920928955078125e-7f;
constexpr float GROUND_FRIC        = 0.5460000634193420f;
constexpr float AIR_FRIC_XZ        = 0.9100000262260437f;
constexpr float Y_DECAY            = 0.9800000190734863f;
constexpr float GRAVITY            = 0.07999999821186066f;
constexpr float LMV_DAMP           = 0.9800000190734863f;

constexpr float PLAYER_SPEED       = 0.10000000149011612f;
constexpr float USING_ITEM_SCALE   = 0.12249999493360519f;
constexpr float AIR_ACCEL_WALK     = 0.019999999552965164f;
constexpr float AIR_ACCEL_SPRINT   = 0.025999998673796654f;
constexpr float SOUL_SAND_FRIC_MUL = 1.2249999046325684f;
constexpr float DEFAULT_SLIP       = 0.6000000238418579f;

struct Vec3 { float x, y, z; };

static void move_relative(Vec3* vel, float yaw_deg,
                          float strafe, float up, float forward, float speed)
{
    const float dist_sq = (float)((float)(strafe * strafe) + (float)(up * up))
                        + (float)(forward * forward);
    if (dist_sq < 0.000099999997f) return;

    const float factor = speed / fmaxf(sqrtf(dist_sq), 1.0f);
    const float scaled_strafe  = strafe   * factor;
    const float scaled_up      = up       * factor;
    const float scaled_forward = forward  * factor;

    const float rad = yaw_deg * DEG_TO_RAD;
    const float sin_yaw = sinf(rad);
    const float cos_yaw = cosf(rad);

    vel->y += scaled_up;
    vel->x += scaled_strafe  * cos_yaw - scaled_forward * sin_yaw;
    vel->z += scaled_forward * cos_yaw + scaled_strafe  * sin_yaw;
}

// mce::Math sin lookup table — 65536-entry table BDS indexes via
// (int)(rad * 10430.378) or (int)(yaw_deg * 182.04443). Used by
// JumpFromGroundSystem's sprint horizontal boost, getViewVector, etc.
// Built with MSVC sinf so bit-exact to BDS's MSVC build.
static float mce_sin_table[65536];
static bool  mce_sin_table_ready = false;
static void mce_init_sin_table() {
    if (mce_sin_table_ready) return;
    for (int i = 0; i < 65536; i++) {
        mce_sin_table[i] = sinf((float)i * (2.0f * 3.14159265358979323846f / 65536.0f));
    }
    mce_sin_table_ready = true;
}

static inline float apply_friction(float v, float k) {
    return (fabsf(v) > BDS_EPSILON) ? v * k : 0.0f;
}

#pragma float_control(push)
#pragma float_control(precise, on)
static float movement_speed(float value, int speed_lvl, int slow_lvl, float freeze_ratio) {
    __m128 v       = _mm_set_ss(value);
    __m128 sp_term = _mm_mul_ss(_mm_set_ss(0.2f),  _mm_set_ss((float)speed_lvl));
    __m128 sl_term = _mm_mul_ss(_mm_set_ss(0.15f), _mm_set_ss((float)slow_lvl));
    __m128 factor  = _mm_add_ss(_mm_set_ss(1.0f), sp_term);
    factor         = _mm_sub_ss(factor, sl_term);
    __m128 speed   = _mm_mul_ss(v, factor);
    if (freeze_ratio > 0.0f) {
        float fr = freeze_ratio > 1.0f ? 1.0f : freeze_ratio;
        __m128 fz_term  = _mm_mul_ss(_mm_set_ss(0.05f), _mm_set_ss(fr));
        __m128 fz_scale = _mm_sub_ss(_mm_set_ss(1.0f), fz_term);
        speed = _mm_mul_ss(speed, fz_scale);
    }
    float out = _mm_cvtss_f32(speed);
    return out > 0.0f ? out : 0.0f;
}

static float friction_influenced_speed(float walk_value, int speed_lvl, int slow_lvl, float freeze_ratio,
                                       bool using_item, bool sprint_active,
                                       bool is_under_water, bool is_in_lava, bool swim_skip_gravity,
                                       bool on_ground, float slipperiness, bool is_soul_sand) {
    float base = movement_speed(walk_value, speed_lvl, slow_lvl, freeze_ratio);
    if (using_item) base = _mm_cvtss_f32(_mm_mul_ss(_mm_set_ss(base), _mm_set_ss(USING_ITEM_SCALE)));
    if (sprint_active) base = _mm_cvtss_f32(_mm_mul_ss(_mm_set_ss(base), _mm_set_ss(1.3f)));

    if (is_under_water || is_in_lava || swim_skip_gravity) return AIR_ACCEL_WALK;
    if (!on_ground) return sprint_active ? AIR_ACCEL_SPRINT : AIR_ACCEL_WALK;

    float slip = slipperiness;
    if (is_soul_sand) slip = _mm_cvtss_f32(_mm_mul_ss(_mm_set_ss(slip), _mm_set_ss(SOUL_SAND_FRIC_MUL)));
    __m128 fric_const = _mm_mul_ss(_mm_set_ss(slip), _mm_set_ss(AIR_FRIC_XZ));
    __m128 r  = _mm_div_ss(_mm_set_ss(GROUND_FRIC), fric_const);
    __m128 r2 = _mm_mul_ss(r, r);
    __m128 r3 = _mm_mul_ss(r2, r);
    return _mm_cvtss_f32(_mm_mul_ss(r3, _mm_set_ss(base)));
}
#pragma float_control(pop)

static inline void move_aabb_axis(float min_in, float max_in, float vel,
                                  float* min_out, float* max_out) {
    *min_out = min_in + vel;
    *max_out = max_in + vel;
}

inline float f(const Napi::CallbackInfo& info, size_t i) {
    return static_cast<float>(info[i].As<Napi::Number>().DoubleValue());
}

inline Napi::Number num(Napi::Env env, float v) {
    return Napi::Number::New(env, static_cast<double>(v));
}

Napi::Value Sinf(const Napi::CallbackInfo& info) {
    return num(info.Env(), sinf(f(info, 0) * DEG_TO_RAD));
}

Napi::Value Cosf(const Napi::CallbackInfo& info) {
    return num(info.Env(), cosf(f(info, 0) * DEG_TO_RAD));
}

Napi::Value Sincosf(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    float angle = f(info, 0) * DEG_TO_RAD;
    Napi::Object r = Napi::Object::New(env);
    r.Set("sin", num(env, sinf(angle)));
    r.Set("cos", num(env, cosf(angle)));
    return r;
}

Napi::Value MoveRelative(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    Vec3 v = { f(info, 0), f(info, 1), f(info, 2) };
    float yaw     = f(info, 3);
    float strafe  = f(info, 4);
    float up      = f(info, 5);
    float forward = f(info, 6);
    float speed   = f(info, 7);
    move_relative(&v, yaw, strafe, up, forward, speed);
    Napi::Object r = Napi::Object::New(env);
    r.Set("x", num(env, v.x));
    r.Set("y", num(env, v.y));
    r.Set("z", num(env, v.z));
    return r;
}

Napi::Value ApplyFriction(const Napi::CallbackInfo& info) {
    return num(info.Env(), apply_friction(f(info, 0), f(info, 1)));
}

Napi::Value MoveAABBAxis(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    float min_in = f(info, 0);
    float max_in = f(info, 1);
    float vel    = f(info, 2);
    float min_out, max_out;
    move_aabb_axis(min_in, max_in, vel, &min_out, &max_out);
    Napi::Object r = Napi::Object::New(env);
    r.Set("min", num(env, min_out));
    r.Set("max", num(env, max_out));
    return r;
}

// mce::g_mSin[idx & 0xFFFF] — BDS sin lookup table indexed by integer.
// Used by JumpFromGroundSystem's sprint boost: idx = (int)(yaw_deg * 182.04443f).
Napi::Value SinTable(const Napi::CallbackInfo& info) {
    mce_init_sin_table();
    int i = info[0].As<Napi::Number>().Int32Value() & 0xFFFF;
    return num(info.Env(), mce_sin_table[i]);
}

Napi::Value Fround(const Napi::CallbackInfo& info) {
    return num(info.Env(), f(info, 0));
}

Napi::Value MovementSpeed(const Napi::CallbackInfo& info) {
    float value = f(info, 0);
    int speed_lvl = info[1].As<Napi::Number>().Int32Value();
    int slow_lvl  = info[2].As<Napi::Number>().Int32Value();
    float freeze  = f(info, 3);
    return num(info.Env(), movement_speed(value, speed_lvl, slow_lvl, freeze));
}

Napi::Value FrictionInfluencedSpeed(const Napi::CallbackInfo& info) {
    float walk_value = f(info, 0);
    int speed_lvl = info[1].As<Napi::Number>().Int32Value();
    int slow_lvl  = info[2].As<Napi::Number>().Int32Value();
    float freeze  = f(info, 3);
    bool using_item     = info[4].As<Napi::Boolean>().Value();
    bool sprint_active  = info[5].As<Napi::Boolean>().Value();
    bool is_under_water = info[6].As<Napi::Boolean>().Value();
    bool is_in_lava     = info[7].As<Napi::Boolean>().Value();
    bool swim_skip      = info[8].As<Napi::Boolean>().Value();
    bool on_ground      = info[9].As<Napi::Boolean>().Value();
    float slip          = f(info, 10);
    bool is_soul_sand   = info[11].As<Napi::Boolean>().Value();
    return num(info.Env(),
        friction_influenced_speed(walk_value, speed_lvl, slow_lvl, freeze,
                                  using_item, sprint_active,
                                  is_under_water, is_in_lava, swim_skip,
                                  on_ground, slip, is_soul_sand));
}

Napi::Object Constants(Napi::Env env) {
    Napi::Object c = Napi::Object::New(env);
    c.Set("DEG_TO_RAD",  num(env, DEG_TO_RAD));
    c.Set("BDS_EPSILON", num(env, BDS_EPSILON));
    c.Set("GROUND_FRIC", num(env, GROUND_FRIC));
    c.Set("AIR_FRIC_XZ", num(env, AIR_FRIC_XZ));
    c.Set("Y_DECAY",     num(env, Y_DECAY));
    c.Set("GRAVITY",     num(env, GRAVITY));
    c.Set("LMV_DAMP",    num(env, LMV_DAMP));
    return c;
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("sinf",          Napi::Function::New(env, Sinf));
    exports.Set("cosf",           Napi::Function::New(env, Cosf));
    exports.Set("sincosf",        Napi::Function::New(env, Sincosf));
    exports.Set("moveRelative",   Napi::Function::New(env, MoveRelative));
    exports.Set("applyFriction",  Napi::Function::New(env, ApplyFriction));
    exports.Set("moveAABBAxis",   Napi::Function::New(env, MoveAABBAxis));
    exports.Set("sinTable",       Napi::Function::New(env, SinTable));
    exports.Set("fround",         Napi::Function::New(env, Fround));
    exports.Set("movementSpeed",  Napi::Function::New(env, MovementSpeed));
    exports.Set("frictionInfluencedSpeed", Napi::Function::New(env, FrictionInfluencedSpeed));
    exports.Set("constants",      Constants(env));
    return exports;
}

}

NODE_API_MODULE(bedrock_native_math, Init)
