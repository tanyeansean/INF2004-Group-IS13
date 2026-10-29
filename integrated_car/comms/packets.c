/**
 * @file    packets.c
 * @brief   Bit-field helpers for the packed MQTT payloads (see packets.h).
 *
 * Plain functions with no SDK calls, so the same code runs in the host tests.
 */
#include "packets.h"

#define BITS_PER_BYTE   (8u)

void pkt_put(uint8_t * p_buf, pkt_field_t field, uint32_t value)
{
    uint32_t i;

    for (i = 0u; i < field.width; i++)
    {
        uint32_t bit  = (uint32_t)field.pos + i;
        uint8_t  mask = (uint8_t)(1u << (bit % BITS_PER_BYTE));

        if (((value >> i) & 1u) != 0u)
        {
            p_buf[bit / BITS_PER_BYTE] |= mask;
        }
        else
        {
            p_buf[bit / BITS_PER_BYTE] &= (uint8_t)~mask;
        }
    }
}

uint32_t pkt_get(uint8_t const * p_buf, pkt_field_t field)
{
    uint32_t value = 0u;
    uint32_t i;

    for (i = 0u; i < field.width; i++)
    {
        uint32_t bit = (uint32_t)field.pos + i;

        if ((p_buf[bit / BITS_PER_BYTE] & (1u << (bit % BITS_PER_BYTE))) != 0u)
        {
            value |= (1u << i);
        }
    }
    return value;
}

int32_t pkt_get_signed(uint8_t const * p_buf, pkt_field_t field)
{
    uint32_t raw  = pkt_get(p_buf, field);
    uint32_t sign = 1u << (field.width - 1u);

    /* sign-extend: (raw ^ sign) - sign */
    return (int32_t)(raw ^ sign) - (int32_t)sign;
}

uint32_t pkt_scale_u(float value, float step, uint32_t max)
{
    float steps;

    if (value != value)                 /* NaN: test before dividing */
    {
        return 0u;
    }
    steps = value / step;
    if (!(steps > 0.0f))
    {
        return 0u;
    }
    if (steps >= (float)max)
    {
        return max;
    }
    return (uint32_t)(steps + 0.5f);    /* round to nearest */
}

int32_t pkt_scale_s(float value, float step, uint8_t width)
{
    int32_t hi = (int32_t)(1u << (width - 1u)) - 1;
    int32_t lo = -hi - 1;
    float   steps;

    if (value != value)                 /* NaN: test before dividing */
    {
        return 0;
    }
    steps = value / step;
    if (steps >= (float)hi)
    {
        return hi;
    }
    if (steps <= (float)lo)
    {
        return lo;
    }
    return (steps >= 0.0f) ? (int32_t)(steps + 0.5f) : (int32_t)(steps - 0.5f);
}

uint32_t pkt_wrap(float value, float step)
{
    /* Keep within int32 before converting (a float -> int conversion out
     * of range is undefined in C); then the low bits wrap naturally. */
    float steps;

    if (value != value)
    {
        return 0u;
    }
    steps = value / step;
    if (steps >= 2.0e9f)
    {
        steps = 2.0e9f;
    }
    if (steps <= -2.0e9f)
    {
        steps = -2.0e9f;
    }
    return (uint32_t)((steps >= 0.0f) ? (int32_t)(steps + 0.5f) : (int32_t)(steps - 0.5f));
}

uint32_t pkt_max(pkt_field_t field)
{
    return (field.width >= 32u) ? 0xFFFFFFFFu : ((1u << field.width) - 1u);
}

uint32_t pkt_sat(uint32_t value, pkt_field_t field)
{
    uint32_t max = pkt_max(field);

    return (value > max) ? max : value;
}
