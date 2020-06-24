#include <cstdlib>
#include <cstring>

#include "base/logging.hh"
#include "sim/numa.hh"

using namespace NUMA;

/** The number of bits held in each basic type element **/
static const long unsigned int BITMASK_NBITS = 8 * sizeof(long unsigned int);

#define BITMASK_NTH(i) ((i) / BITMASK_NBITS)
#define BITMASK_ITH(i) (((i) % BITMASK_NBITS))

const long unsigned int Bitmask::MAXBITS;

Bitmask::Bitmask() {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = 0;
}

Bitmask::Bitmask(const size_t n) {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = 0;
    set(n);
}

Bitmask::Bitmask(const size_t i, const size_t ii) {
    for (int j = 0; j < Bitmask::NULONGS; j++) mask[j] = 0;
    set_range(i, ii);
}

Bitmask::Bitmask(const Bitmask &src) {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = src.mask[i];
}

Bitmask::Bitmask(const char *str) {
    Bitmask b;
    int type = 0;  // 0 = 0110010, 1 = 3,4,8,7,6,5
    size_t len = strlen(str);

    if (str == NULL || !strcasecmp(str, "none") || !strcasecmp(str, "zero") ||
        !strcasecmp(str, "empty")) {
        zero();
        return;
    }

    if (!strcasecmp(str, "all") || !strcasecmp(str, "fill") ||
        !strcasecmp(str, "full")) {
        fill();
        return;
    }

    // Empty mask.
    zero();

    /* scan for string type */
    for (size_t i = 0; i < len; i++) {
        if (str[i] == '0' || str[i] == '1') continue;

        if (str[i] == '2' || str[i] == '3' || str[i] == '4' || str[i] == '5' ||
            str[i] == '6' || str[i] == '7' || str[i] == '8' || str[i] == '9' ||
            str[i] == ',')
            type = 1;
        else {
            type = -1;
            break;
        }
    }

    switch (type) {
        case 0:
            goto binary_mask;
        case 1:
            goto num_list_mask;
        default:
            fatal("Invalid bitmask string %s.", str);
    }

binary_mask:
    for (size_t i = len - 1; i < len && i < Bitmask::NULONGS; i++)
        if (str[len - i - 1] == '1') set(i);
    return;

num_list_mask:
    char *saveptr, *tok, *strd = strdup(str);
    tok = strtok_r(strd, ",", &saveptr);
    while (tok != NULL) {
        if (set(strtoul(tok, NULL, 10)) == -1) {
            free(strd);
            fatal("Invalid bit number %s. MAXBITS is %lu", tok,
                  Bitmask::MAXBITS);
        }
        tok = strtok_r(NULL, ",", &saveptr);
    }
    free(strd);
    return;
}

int Bitmask::bitwise_cmp(const Bitmask &rhs) const {
    for (int i = Bitmask::NULONGS - 1; i >= 0; i--) {
        if (mask[i] > rhs.mask[i]) return 1;
        if (mask[i] < rhs.mask[i]) return -1;
    }
    return 0;
}

Bitmask Bitmask::bitwise_or(const Bitmask &rhs) const {
    Bitmask ret(rhs);
    for (int i = 0; i < Bitmask::NULONGS; i++)
        ret.mask[i] = mask[i] | rhs.mask[i];
    return ret;
}

Bitmask Bitmask::bitwise_and(const Bitmask &rhs) const {
    Bitmask ret(rhs);
    for (int i = 0; i < Bitmask::NULONGS; i++)
        ret.mask[i] = mask[i] & rhs.mask[i];
    return ret;
}

Bitmask Bitmask::bitwise_not() const {
    Bitmask ret = Bitmask();
    for (int i = 0; i < Bitmask::NULONGS; i++) ret.mask[i] = ~mask[i];
    return ret;
}

void Bitmask::copy_to(Bitmask &dst) const {
    for (int i = 0; i < Bitmask::NULONGS; i++) dst.mask[i] = mask[i];
}

void Bitmask::copy_from(const Bitmask &src) {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = src.mask[i];
}

void Bitmask::zero() {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = 0;
}

void Bitmask::fill() {
    for (int i = 0; i < Bitmask::NULONGS; i++) mask[i] = ~0UL;
}

int Bitmask::isset(const long unsigned int i) const {
    return (i >= Bitmask::MAXBITS)
               ? -1
               : (mask[BITMASK_NTH(i)] & (1UL << BITMASK_ITH(i))) > 0UL;
}

bool Bitmask::iszero() const {
    for (unsigned int i = 0; i < Bitmask::NULONGS; i++)
        if (mask[i] != 0) return false;
    return true;
}

bool Bitmask::isfull() const {
    for (unsigned int i = 0; i < Bitmask::NULONGS; i++)
        if (mask[i] != ~0UL) return false;
    return true;
}

int Bitmask::set(const long unsigned int i) {
    if (i >= Bitmask::MAXBITS) return -1;
    mask[BITMASK_NTH(i)] |= (1UL << BITMASK_ITH(i));
    return 0;
}

int Bitmask::clear(const long unsigned int i) {
    if (i >= Bitmask::MAXBITS) return -1;
    mask[BITMASK_NTH(i)] &= ~(1UL << BITMASK_ITH(i));
    return 0;
}

int Bitmask::set_range(const long unsigned int i, const long unsigned int ii) {
    if (i == ii) return set(i);
    if (i >= Bitmask::MAXBITS || ii >= Bitmask::MAXBITS || i > ii) return -1;

    long unsigned int k = BITMASK_ITH(ii + 1);
    long unsigned int low = ((~0UL) << BITMASK_ITH(i));
    long unsigned int n = BITMASK_NTH(i);
    long unsigned int nn = BITMASK_NTH(ii);
    long unsigned int high = k == 0 ? (~0UL) : ~((~0UL) << k);

    if (nn > n) {
        for (k = n + 1; k <= nn - 1; k++) mask[k] = (~0UL);
        mask[n] |= low;
        mask[nn] |= high;
    } else
        mask[n] |= (low & high);

    return 0;
}

int Bitmask::clear_range(const long unsigned int i,
                         const long unsigned int ii) {
    if (i == ii) return clear(i);
    if (i >= Bitmask::MAXBITS || ii >= Bitmask::MAXBITS || i > ii) return -1;

    long unsigned int k = BITMASK_ITH(ii + 1);
    long unsigned int low = ~((~0UL) << BITMASK_ITH(i));
    long unsigned int n = BITMASK_NTH(i);
    long unsigned int nn = BITMASK_NTH(ii);
    long unsigned int high = k == 0 ? 0UL : ((~0UL) << k);

    if (nn > n) {
        for (k = n + 1; k <= nn - 1; k++) mask[k] = 0UL;
        mask[n] &= low;
        mask[nn] &= high;
    } else
        mask[n] &= (low | high);

    return 0;
}

long unsigned int Bitmask::nset() const {
    long unsigned int test = 1UL;
    long unsigned int nset = 0;

    for (long unsigned int n = 0; n < Bitmask::NULONGS; n++) {
        long unsigned int b = mask[n];
        for (long unsigned int i = 0; i < BITMASK_NBITS; i++) {
            nset += b & test ? 1 : 0;
            b = b >> 1;
        }
    }
    return nset;
}

long Bitmask::last() const {
    long n;
    unsigned int i = 0;

    for (n = Bitmask::NULONGS - 1; n >= 0 && mask[n] == 0; n--)
        ;
    if (n < 0) return -1;

    long unsigned int m = mask[n];

    for (i = 0; i < BITMASK_NBITS && m; i++) m = m >> 1;
    return (BITMASK_NBITS * n) + i - 1;
}

long Bitmask::first() const {
    unsigned int n, i = 0;

    for (n = 0; n < Bitmask::NULONGS && mask[n] == 0; n++)
        ;
    if (n == Bitmask::NULONGS) return -1;

    long unsigned int m = mask[n];

    for (i = 0; i < BITMASK_NBITS && m; i++) m = m << 1;

    int res = (BITMASK_NBITS * n) + BITMASK_NBITS - i;
    return res;
}

long Bitmask::next(long n) const {
    const long f = first();
    const long l = last();

    if (n < f || n >= l) return f;
    while (!isset(n)) n++;
    return n;
}

char *Bitmask::to_string() const {
    size_t i, len = Bitmask::MAXBITS + 1;
    char *output = (char *)malloc(len);

    if (output == NULL) return NULL;
    memset(output, 0, len);

    for (i = 0; i < Bitmask::MAXBITS; i++)
        output[i] = isset((int)(i - 1)) ? '1' : '0';
    return output;
}

bool operator==(const Bitmask &lhs, const Bitmask &rhs) {
    return lhs.bitwise_cmp(rhs) == 0;
}

Bitmask Bitmask::operator|(const Bitmask &rhs) const {
    return bitwise_or(rhs);
}

Bitmask Bitmask::operator&(const Bitmask &rhs) const {
    return bitwise_and(rhs);
}

Bitmask &Bitmask::operator|=(const Bitmask &rhs) {
    copy_from(bitwise_or(rhs));
    return *this;
}

Bitmask &Bitmask::operator&=(const Bitmask &rhs) {
    copy_from(bitwise_and(rhs));
    return *this;
}

Bitmask Bitmask::operator~() const { return bitwise_not(); }
Bitmask Bitmask::operator!() const { return !iszero(); }

Bitmask::operator bool() const { return iszero(); }
