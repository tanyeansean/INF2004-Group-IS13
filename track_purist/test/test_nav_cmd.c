/**
 * @file test_nav_cmd.c
 * @brief Host tests: MQTT letter->movement mapping and the demo flow
 *        (barcode stored -> executed once at the next cross-junction).
 */
#include "nav_cmd.h"
#include "test_util.h"

#define L(ch)  ((uint8_t)((ch) - 'A'))

static line_err_t line_plain(void)    { return line_err_pack(0, 0u, 0u); }
static line_err_t line_junction(void) { return line_err_pack(0, 1u, 0u); }

/* Mapping order is BARCMD_*: left, right, forward, u-turn. */
static void apply_map(nav_map_t *m, char left, char right, char fwd, char uturn)
{
    uint8_t letters[BARCMD_COUNT];
    uint8_t p[NAV_MAP_PAYLOAD_LEN];
    letters[BARCMD_LEFT]     = L(left);
    letters[BARCMD_RIGHT]    = L(right);
    letters[BARCMD_STRAIGHT] = L(fwd);
    letters[BARCMD_U_TURN]   = L(uturn);
    (void)nav_map_encode(letters, p);
    (void)nav_map_apply(m, p, NAV_MAP_PAYLOAD_LEN);
}

int main(void)
{
    printf("== nav_cmd tests ==\n");

    /* --- mapping: starts empty --- */
    {
        nav_map_t m; uint8_t mv;
        nav_map_init(&m);
        CHECK(!nav_map_is_set(nav_map_snapshot(&m)), "no mapping at boot");
        CHECK(!nav_map_lookup(nav_map_snapshot(&m), L('A'), &mv), "lookup fails with no mapping");
    }

    /* --- mapping: encode/apply round trip, any letters --- */
    {
        nav_map_t m; uint8_t mv = 0xFFu; uint32_t s;
        nav_map_init(&m);
        apply_map(&m, 'F', 'Z', 'W', 'X');
        s = nav_map_snapshot(&m);
        CHECK(nav_map_is_set(s), "mapping set after apply");
        CHECK(nav_map_lookup(s, L('F'), &mv) && mv == BARCMD_LEFT,     "F -> LEFT");
        CHECK(nav_map_lookup(s, L('Z'), &mv) && mv == BARCMD_RIGHT,    "Z -> RIGHT");
        CHECK(nav_map_lookup(s, L('W'), &mv) && mv == BARCMD_STRAIGHT, "W -> FORWARD");
        CHECK(nav_map_lookup(s, L('X'), &mv) && mv == BARCMD_U_TURN,   "X -> U_TURN");
        CHECK(!nav_map_lookup(s, L('A'), &mv), "A unmapped");
    }

    /* --- mapping: exact wire bytes (documented layout) --- */
    {
        uint8_t letters[BARCMD_COUNT] = { 0u, 1u, 2u, 3u }; /* A B C D */
        uint8_t p[NAV_MAP_PAYLOAD_LEN];
        CHECK_EQ_INT(nav_map_encode(letters, p), NAV_MAP_OK, "encode ok");
        /* word = 0 | 1<<5 | 2<<10 | 3<<15 = 0x018820 */
        CHECK_EQ_INT(p[0], 0x20, "payload[0] = 0x20");
        CHECK_EQ_INT(p[1], 0x88, "payload[1] = 0x88");
        CHECK_EQ_INT(p[2], 0x01, "payload[2] = 0x01");
    }

    /* --- mapping: bad payloads rejected, old mapping kept --- */
    {
        nav_map_t m; uint8_t mv = 0xFFu; uint32_t before;
        uint8_t dup[3], big[3] = { 0x1Au, 0x00u, 0x00u }; /* left letter 26 */
        uint8_t rsv[3] = { 0x20u, 0x88u, 0x11u };          /* bit 20 set */
        uint8_t letters[BARCMD_COUNT] = { 5u, 5u, 6u, 7u };
        nav_map_init(&m);
        apply_map(&m, 'A', 'B', 'C', 'D');
        before = nav_map_snapshot(&m);

        CHECK_EQ_INT(nav_map_apply(&m, big, 2u), NAV_MAP_ERR_LEN, "2-byte payload -> ERR_LEN");
        CHECK_EQ_INT(nav_map_apply(&m, big, 3u), NAV_MAP_ERR_RANGE, "letter 26 -> ERR_RANGE");
        CHECK_EQ_INT(nav_map_apply(&m, rsv, 3u), NAV_MAP_ERR_RANGE, "reserved bits -> ERR_RANGE");
        CHECK_EQ_INT(nav_map_encode(letters, dup), NAV_MAP_ERR_DUP, "encode duplicate -> ERR_DUP");
        dup[0] = 0xA5u; dup[1] = 0x98u; dup[2] = 0x03u; /* F,F,G,H = 0x0398A5 */
        CHECK_EQ_INT(nav_map_apply(&m, dup, 3u), NAV_MAP_ERR_DUP, "apply duplicate -> ERR_DUP");
        CHECK_EQ_INT(nav_map_snapshot(&m), before, "rejected payloads keep old mapping");
        CHECK(nav_map_lookup(nav_map_snapshot(&m), L('C'), &mv) && mv == BARCMD_STRAIGHT,
              "old mapping still works");
    }

    /* --- mapping can be replaced (instructor resends) --- */
    {
        nav_map_t m; uint8_t mv = 0xFFu;
        nav_map_init(&m);
        apply_map(&m, 'A', 'B', 'C', 'D');
        apply_map(&m, 'D', 'C', 'B', 'A');
        CHECK(nav_map_lookup(nav_map_snapshot(&m), L('A'), &mv) && mv == BARCMD_U_TURN,
              "second mapping replaces the first");
    }

    /* --- demo flow: barcode stored, executed once at next junction --- */
    {
        nav_map_t m; nav_state_t s;
        uint8_t mv = 0xFFu, let = 0xFFu, ev;
        nav_map_init(&m);
        apply_map(&m, 'F', 'Z', 'W', 'X');
        nav_reset(&s);

        nav_on_barcode(&s, barcode_pack_letter(L('Z')));
        CHECK(nav_has_pending(&s, &let) && let == L('Z'), "Z stored as pending");

        ev = nav_on_line(&s, line_plain(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_NONE, "no junction -> keep driving");

        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_MOVE, "junction -> move");
        CHECK_EQ_INT(mv, BARCMD_RIGHT, "Z -> RIGHT at junction");
        CHECK_EQ_INT(let, L('Z'), "reports letter Z");
        CHECK(!nav_has_pending(&s, 0), "pending cleared after execution");

        /* junction flag stays high for several messages: only fires once */
        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_NONE, "held junction does not re-fire");

        /* next barcode, next junction */
        (void)nav_on_line(&s, line_plain(), nav_map_snapshot(&m), &mv, &let);
        nav_on_barcode(&s, barcode_pack_letter(L('W')));
        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK(ev == NAV_EVT_MOVE && mv == BARCMD_STRAIGHT, "second barcode W -> FORWARD");
    }

    /* --- junction with nothing pending, invalid barcode, latest wins --- */
    {
        nav_map_t m; nav_state_t s;
        uint8_t mv = 0xFFu, let = 0xFFu, ev;
        nav_map_init(&m);
        apply_map(&m, 'A', 'B', 'C', 'D');
        nav_reset(&s);

        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_NONE, "junction without barcode -> drive on");
        (void)nav_on_line(&s, line_plain(), nav_map_snapshot(&m), &mv, &let);

        nav_on_barcode(&s, barcode_invalid());
        CHECK(!nav_has_pending(&s, 0), "INVALID barcode not stored");

        nav_on_barcode(&s, barcode_pack_letter(L('A')));
        nav_on_barcode(&s, barcode_invalid());
        CHECK(nav_has_pending(&s, &let) && let == L('A'), "INVALID does not erase pending A");

        nav_on_barcode(&s, barcode_pack_letter(L('B')));
        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK(ev == NAV_EVT_MOVE && mv == BARCMD_RIGHT, "latest barcode before junction wins");
    }

    /* --- letter not in the mapping / no mapping received --- */
    {
        nav_map_t m; nav_state_t s;
        uint8_t mv = 0xFFu, let = 0xFFu, ev;
        nav_map_init(&m);
        nav_reset(&s);
        nav_on_barcode(&s, barcode_pack_letter(L('Q')));
        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_UNMAPPED, "no mapping yet -> UNMAPPED");
        CHECK_EQ_INT(let, L('Q'), "UNMAPPED still reports the letter");

        apply_map(&m, 'A', 'B', 'C', 'D');
        (void)nav_on_line(&s, line_plain(), nav_map_snapshot(&m), &mv, &let);
        nav_on_barcode(&s, barcode_pack_letter(L('Q')));
        ev = nav_on_line(&s, line_junction(), nav_map_snapshot(&m), &mv, &let);
        CHECK_EQ_INT(ev, NAV_EVT_UNMAPPED, "letter outside mapping -> UNMAPPED");
        CHECK(!nav_has_pending(&s, 0), "UNMAPPED also clears pending");
    }

    return TEST_SUMMARY();
}
