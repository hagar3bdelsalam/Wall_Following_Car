#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>

// BIT HELPERS 
#define SBI(reg, bit)   ((reg) |=  (1 << (bit)))
#define CBI(reg, bit)   ((reg) &= ~(1 << (bit)))
#define RBI(reg, bit)   (((reg) >> (bit)) & 1)

// CPU FREQUENCY (must match board definition)
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#endif // UTILS_H
