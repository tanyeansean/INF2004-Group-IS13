/**
 * @file nav_cmd.c
 * @brief Navigation command interface (Buddy 3). Pure logic, host-tested.
 */
#include "nav_cmd.h"

/* ------------------------------------------------------------------ */
/* Mapping                                                             */
/* ------------------------------------------------------------------ */
static uint8_t map_letter(uint32_t word, uint8_t move)
{
    return (uint8_t)((word >> (move * NAV_MAP_LETTER_BITS)) & NAV_MAP_LETTER_MASK);
}

void nav_map_init(nav_map_t *m)
{
    m->packed = 0u; /* valid bit clear => no mapping yet */
}

/* Validate 20 bits of letters: each 0..25 and all distinct. */
static uint8_t check_letters(uint32_t word)
{
    uint8_t i, j;
    for (i = 0u; i < BARCMD_COUNT; i++) {
        if (map_letter(word, i) >= BARCODE_LETTER_COUNT) {
            return NAV_MAP_ERR_RANGE;
        }
    }
    for (i = 0u; i < BARCMD_COUNT; i++) {
        for (j = (uint8_t)(i + 1u); j < BARCMD_COUNT; j++) {
            if (map_letter(word, i) == map_letter(word, j)) {
                return NAV_MAP_ERR_DUP;
            }
        }
    }
    return NAV_MAP_OK;
}

uint8_t nav_map_apply(nav_map_t *m, const uint8_t *payload, uint32_t len)
{
    uint32_t word;
    uint8_t  st;

    if (payload == 0 || len != NAV_MAP_PAYLOAD_LEN) {
        return NAV_MAP_ERR_LEN;
    }
    word = (uint32_t)payload[0]
         | ((uint32_t)payload[1] << 8)
         | ((uint32_t)payload[2] << 16);
    if ((word & 0xF00000uL) != 0u) {
        return NAV_MAP_ERR_RANGE; /* reserved bits must be 0 */
    }
    st = check_letters(word);
    if (st != NAV_MAP_OK) {
        return st; /* keep the previous mapping */
    }
    /* Build the complete value locally, then publish with ONE store. */
    m->packed = word | NAV_MAP_VALID_BIT;
    return NAV_MAP_OK;
}

uint32_t nav_map_snapshot(const nav_map_t *m)
{
    return m->packed; /* single aligned 32-bit load */
}

uint8_t nav_map_is_set(uint32_t snapshot)
{
    return (uint8_t)((snapshot & NAV_MAP_VALID_BIT) ? 1u : 0u);
}

uint8_t nav_map_lookup(uint32_t snapshot, uint8_t letter, uint8_t *move)
{
    uint8_t i;
    if (!nav_map_is_set(snapshot) || letter >= BARCODE_LETTER_COUNT) {
        return 0u;
    }
    for (i = 0u; i < BARCMD_COUNT; i++) {
        if (map_letter(snapshot, i) == letter) {
            *move = i;
            return 1u;
        }
    }
    return 0u;
}

uint8_t nav_map_encode(const uint8_t letters[BARCMD_COUNT],
                       uint8_t payload[NAV_MAP_PAYLOAD_LEN])
{
    uint32_t word = 0u;
    uint8_t  i, st;
    for (i = 0u; i < BARCMD_COUNT; i++) {
        if (letters[i] >= BARCODE_LETTER_COUNT) {
            return NAV_MAP_ERR_RANGE;
        }
        word |= (uint32_t)letters[i] << (i * NAV_MAP_LETTER_BITS);
    }
    st = check_letters(word);
    if (st != NAV_MAP_OK) {
        return st;
    }
    payload[0] = (uint8_t)(word & 0xFFu);
    payload[1] = (uint8_t)((word >> 8) & 0xFFu);
    payload[2] = (uint8_t)((word >> 16) & 0xFFu);
    return NAV_MAP_OK;
}

/* ------------------------------------------------------------------ */
/* Pending barcode + junction trigger                                  */
/* ------------------------------------------------------------------ */
void nav_reset(nav_state_t *s)
{
    s->pending = barcode_invalid();
    s->prev_junction = 0u;
}

void nav_on_barcode(nav_state_t *s, barcode_t b)
{
    if (barcode_is_valid(b)) {
        s->pending = b; /* latest valid barcode before the junction wins */
    }
}

uint8_t nav_has_pending(const nav_state_t *s, uint8_t *letter)
{
    if (!barcode_is_valid(s->pending)) {
        return 0u;
    }
    if (letter != 0) {
        *letter = barcode_get_letter(s->pending);
    }
    return 1u;
}

uint8_t nav_on_line(nav_state_t *s, line_err_t le, uint32_t map_snapshot,
                    uint8_t *move, uint8_t *letter)
{
    uint8_t junction = line_err_get_junction(le);
    uint8_t rising   = (uint8_t)(junction && !s->prev_junction);
    uint8_t l;

    s->prev_junction = junction;

    if (!rising || !barcode_is_valid(s->pending)) {
        return NAV_EVT_NONE;
    }

    l = barcode_get_letter(s->pending);
    s->pending = barcode_invalid(); /* each barcode executes once */
    if (letter != 0) {
        *letter = l;
    }
    if (nav_map_lookup(map_snapshot, l, move)) {
        return NAV_EVT_MOVE;
    }
    return NAV_EVT_UNMAPPED;
}
