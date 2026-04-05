#ifndef _RAMMEL_H_
#define _RAMMEL_H_

typedef unsigned int uint;

#define count_of(a) (sizeof(a)/sizeof(*(a)))

#define clamp(v, i, s) ({__typeof__(v) v_ = (v); __typeof__(v) i_ = (i); __typeof__(v) s_ = (s); v_ < i_ ? i_ : v_ > s_ ? s_ : v_;})
#define min(a,b) ({ __typeof__(a) a_=(a); __typeof__(b) b_=(b); a_ < b_ ? a_ : b_;})
#define max(a,b) ({ __typeof__(a) a_=(a); __typeof__(b) b_=(b); a_ > b_ ? a_ : b_;})

#define sqr(a) ({ __typeof__(a) a_=(a); a_*a_;})

static inline int modulo(int a, int b) {
    a = a % b;
    if (a < 0) {
        a += b;
    }
    return a;
}


#endif
