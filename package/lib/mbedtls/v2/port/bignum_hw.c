/*
 *  Multi-precision integer library
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

/*
 *  The following sources were referenced in the design of this Multi-precision
 *  Integer library:
 *
 *  [1] Handbook of Applied Cryptography - 1997
 *      Menezes, van Oorschot and Vanstone
 *
 *  [2] Multi-Precision Math
 *      Tom St Denis
 *      https://github.com/libtom/libtommath/blob/develop/tommath.pdf
 *
 *  [3] GNU Multi-Precision Arithmetic Library
 *      https://gmplib.org/manual/index.html
 *
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_BIGNUM_C)

#include "mbedtls/bignum.h"
#include "mbedtls/bn_mul.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"
#include "drv_sec.h"

#include <string.h>

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_printf     printf
#define mbedtls_calloc    calloc
#define mbedtls_free       free
#endif

#define MPI_VALIDATE_RET( cond )                                       \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_MPI_BAD_INPUT_DATA )
#define MPI_VALIDATE( cond )                                           \
    MBEDTLS_INTERNAL_VALIDATE( cond )

#define ciL    (sizeof(mbedtls_mpi_uint))         /* chars in limb  */
#define biL    (ciL << 3)               /* bits  in limb  */
#define biH    (ciL << 2)               /* half limb size */

#define MPI_SIZE_T_MAX  ( (size_t) -1 ) /* SIZE_T_MAX is not standard */

/*
 * Convert between bits/chars and number of limbs
 * Divide first in order to avoid potential overflows
 */
#define BITS_TO_LIMBS(i)  ( (i) / biL + ( (i) % biL != 0 ) )
#define CHARS_TO_LIMBS(i) ( (i) / ciL + ( (i) % ciL != 0 ) )

/* Implementation that should never be optimized out by the compiler */
static void mbedtls_mpi_zeroize( mbedtls_mpi_uint *v, size_t n )
{
    mbedtls_platform_zeroize( v, ciL * n );
}

/*
 * Initialize one MPI
 */
void mbedtls_mpi_init( mbedtls_mpi *X )
{
    MPI_VALIDATE( X != NULL );

    X->s = 1;
    X->n = 0;
    X->p = NULL;
}

/*
 * Unallocate one MPI
 */
void mbedtls_mpi_free( mbedtls_mpi *X )
{
    if( X == NULL )
        return;

    if( X->p != NULL )
    {
        mbedtls_mpi_zeroize( X->p, X->n );
        mbedtls_free( X->p );
    }

    X->s = 1;
    X->n = 0;
    X->p = NULL;
}

/*
 * Enlarge to the specified number of limbs
 */
int mbedtls_mpi_grow( mbedtls_mpi *X, size_t nblimbs )
{
    mbedtls_mpi_uint *p;
    MPI_VALIDATE_RET( X != NULL );

    if( nblimbs > MBEDTLS_MPI_MAX_LIMBS )
        return( MBEDTLS_ERR_MPI_ALLOC_FAILED );

    if( X->n < nblimbs )
    {
        if( ( p = (mbedtls_mpi_uint*)mbedtls_calloc( nblimbs, ciL ) ) == NULL )
            return( MBEDTLS_ERR_MPI_ALLOC_FAILED );

        if( X->p != NULL )
        {
            memcpy( p, X->p, X->n * ciL );
            mbedtls_mpi_zeroize( X->p, X->n );
            mbedtls_free( X->p );
        }

        X->n = nblimbs;
        X->p = p;
    }

    return( 0 );
}

/*
 * Resize down as much as possible,
 * while keeping at least the specified number of limbs
 */
int mbedtls_mpi_shrink( mbedtls_mpi *X, size_t nblimbs )
{
    mbedtls_mpi_uint *p;
    size_t i;
    MPI_VALIDATE_RET( X != NULL );

    if( nblimbs > MBEDTLS_MPI_MAX_LIMBS )
        return( MBEDTLS_ERR_MPI_ALLOC_FAILED );

    /* Actually resize up if there are currently fewer than nblimbs limbs. */
    if( X->n <= nblimbs )
        return( mbedtls_mpi_grow( X, nblimbs ) );
    /* After this point, then X->n > nblimbs and in particular X->n > 0. */

    for( i = X->n - 1; i > 0; i-- )
        if( X->p[i] != 0 )
            break;
    i++;

    if( i < nblimbs )
        i = nblimbs;

    if( ( p = (mbedtls_mpi_uint*)mbedtls_calloc( i, ciL ) ) == NULL )
        return( MBEDTLS_ERR_MPI_ALLOC_FAILED );

    if( X->p != NULL )
    {
        memcpy( p, X->p, i * ciL );
        mbedtls_mpi_zeroize( X->p, X->n );
        mbedtls_free( X->p );
    }

    X->n = i;
    X->p = p;

    return( 0 );
}

/*
 * Copy the contents of Y into X
 */
int mbedtls_mpi_copy( mbedtls_mpi *X, const mbedtls_mpi *Y )
{
    int ret = 0;
    size_t i;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );

    if( X == Y )
        return( 0 );

    if( Y->n == 0 )
    {
        mbedtls_mpi_free( X );
        return( 0 );
    }

    for( i = Y->n - 1; i > 0; i-- )
        if( Y->p[i] != 0 )
            break;
    i++;

    X->s = Y->s;

    if( X->n < i )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, i ) );
    }
    else
    {
        memset( X->p + i, 0, ( X->n - i ) * ciL );
    }

    memcpy( X->p, Y->p, i * ciL );

cleanup:

    return( ret );
}

/*
 * Swap the contents of X and Y
 */
void mbedtls_mpi_swap( mbedtls_mpi *X, mbedtls_mpi *Y )
{
    mbedtls_mpi T;
    MPI_VALIDATE( X != NULL );
    MPI_VALIDATE( Y != NULL );

    memcpy( &T,  X, sizeof( mbedtls_mpi ) );
    memcpy(  X,  Y, sizeof( mbedtls_mpi ) );
    memcpy(  Y, &T, sizeof( mbedtls_mpi ) );
}

/*
 * Conditionally assign dest = src, without leaking information
 * about whether the assignment was made or not.
 * dest and src must be arrays of limbs of size n.
 * assign must be 0 or 1.
 */
static void mpi_safe_cond_assign( size_t n,
                                  mbedtls_mpi_uint *dest,
                                  const mbedtls_mpi_uint *src,
                                  unsigned char assign )
{
    size_t i;
    for( i = 0; i < n; i++ )
        dest[i] = dest[i] * ( 1 - assign ) + src[i] * assign;
}

/*
 * Conditionally assign X = Y, without leaking information
 * about whether the assignment was made or not.
 * (Leaking information about the respective sizes of X and Y is ok however.)
 */
int mbedtls_mpi_safe_cond_assign( mbedtls_mpi *X, const mbedtls_mpi *Y, unsigned char assign )
{
    int ret = 0;
    size_t i;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );

    /* make sure assign is 0 or 1 in a time-constant manner */
    assign = (assign | (unsigned char)-assign) >> 7;

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, Y->n ) );

    X->s = X->s * ( 1 - assign ) + Y->s * assign;

    mpi_safe_cond_assign( Y->n, X->p, Y->p, assign );

    for( i = Y->n; i < X->n; i++ )
        X->p[i] *= ( 1 - assign );

cleanup:
    return( ret );
}

/*
 * Conditionally swap X and Y, without leaking information
 * about whether the swap was made or not.
 * Here it is not ok to simply swap the pointers, which whould lead to
 * different memory access patterns when X and Y are used afterwards.
 */
int mbedtls_mpi_safe_cond_swap( mbedtls_mpi *X, mbedtls_mpi *Y, unsigned char swap )
{
    int ret, s;
    size_t i;
    mbedtls_mpi_uint tmp;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );

    if( X == Y )
        return( 0 );

    /* make sure swap is 0 or 1 in a time-constant manner */
    swap = (swap | (unsigned char)-swap) >> 7;

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, Y->n ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( Y, X->n ) );

    s = X->s;
    X->s = X->s * ( 1 - swap ) + Y->s * swap;
    Y->s = Y->s * ( 1 - swap ) +    s * swap;


    for( i = 0; i < X->n; i++ )
    {
        tmp = X->p[i];
        X->p[i] = X->p[i] * ( 1 - swap ) + Y->p[i] * swap;
        Y->p[i] = Y->p[i] * ( 1 - swap ) +     tmp * swap;
    }

cleanup:
    return( ret );
}

/*
 * Set value from integer
 */
int mbedtls_mpi_lset( mbedtls_mpi *X, mbedtls_mpi_sint z )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    MPI_VALIDATE_RET( X != NULL );

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, 1 ) );
    memset( X->p, 0, X->n * ciL );

    X->p[0] = ( z < 0 ) ? -z : z;
    X->s    = ( z < 0 ) ? -1 : 1;

cleanup:

    return( ret );
}

/*
 * Get a specific bit
 */
int mbedtls_mpi_get_bit( const mbedtls_mpi *X, size_t pos )
{
    MPI_VALIDATE_RET( X != NULL );

    if( X->n * biL <= pos )
        return( 0 );

    return( ( X->p[pos / biL] >> ( pos % biL ) ) & 0x01 );
}

/* Get a specific byte, without range checks. */
#define GET_BYTE( X, i )                                \
    ( ( ( X )->p[( i ) / ciL] >> ( ( ( i ) % ciL ) * 8 ) ) & 0xff )

/*
 * Set a bit to a specific value of 0 or 1
 */
int mbedtls_mpi_set_bit( mbedtls_mpi *X, size_t pos, unsigned char val )
{
    int ret = 0;
    size_t off = pos / biL;
    size_t idx = pos % biL;
    MPI_VALIDATE_RET( X != NULL );

    if( val != 0 && val != 1 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    if( X->n * biL <= pos )
    {
        if( val == 0 )
            return( 0 );

        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, off + 1 ) );
    }

    X->p[off] &= ~( (mbedtls_mpi_uint) 0x01 << idx );
    X->p[off] |= (mbedtls_mpi_uint) val << idx;

cleanup:

    return( ret );
}

/*
 * Return the number of less significant zero-bits
 */
size_t mbedtls_mpi_lsb( const mbedtls_mpi *X )
{
    size_t i, j, count = 0;
    MBEDTLS_INTERNAL_VALIDATE_RET( X != NULL, 0 );

    for( i = 0; i < X->n; i++ )
        for( j = 0; j < biL; j++, count++ )
            if( ( ( X->p[i] >> j ) & 1 ) != 0 )
                return( count );

    return( 0 );
}

/*
 * Count leading zero bits in a given integer
 */
static size_t mbedtls_clz( const mbedtls_mpi_uint x )
{
    size_t j;
    mbedtls_mpi_uint mask = (mbedtls_mpi_uint) 1 << (biL - 1);

    for( j = 0; j < biL; j++ )
    {
        if( x & mask ) break;

        mask >>= 1;
    }

    return j;
}

/*
 * Return the number of bits
 */
size_t mbedtls_mpi_bitlen( const mbedtls_mpi *X )
{
    size_t i, j;

    if( X->n == 0 )
        return( 0 );

    for( i = X->n - 1; i > 0; i-- )
        if( X->p[i] != 0 )
            break;

    j = biL - mbedtls_clz( X->p[i] );

    return( ( i * biL ) + j );
}

/*
 * Return the total size in bytes
 */
size_t mbedtls_mpi_size( const mbedtls_mpi *X )
{
    return( ( mbedtls_mpi_bitlen( X ) + 7 ) >> 3 );
}

/*
 * Convert an ASCII character to digit value
 */
static int mpi_get_digit( mbedtls_mpi_uint *d, int radix, char c )
{
    *d = 255;

    if( c >= 0x30 && c <= 0x39 ) *d = c - 0x30;
    if( c >= 0x41 && c <= 0x46 ) *d = c - 0x37;
    if( c >= 0x61 && c <= 0x66 ) *d = c - 0x57;

    if( *d >= (mbedtls_mpi_uint) radix )
        return( MBEDTLS_ERR_MPI_INVALID_CHARACTER );

    return( 0 );
}

/*
 * Import from an ASCII string
 */
int mbedtls_mpi_read_string( mbedtls_mpi *X, int radix, const char *s )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t i, j, slen, n;
    mbedtls_mpi_uint d;
    mbedtls_mpi T;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( s != NULL );

    if( radix < 2 || radix > 16 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    mbedtls_mpi_init( &T );

    slen = strlen( s );

    if( radix == 16 )
    {
        if( slen > MPI_SIZE_T_MAX >> 2 )
            return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

        n = BITS_TO_LIMBS( slen << 2 );

        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, n ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

        for( i = slen, j = 0; i > 0; i--, j++ )
        {
            if( i == 1 && s[i - 1] == '-' )
            {
                X->s = -1;
                break;
            }

            MBEDTLS_MPI_CHK( mpi_get_digit( &d, radix, s[i - 1] ) );
            X->p[j / ( 2 * ciL )] |= d << ( ( j % ( 2 * ciL ) ) << 2 );
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

        for( i = 0; i < slen; i++ )
        {
            if( i == 0 && s[i] == '-' )
            {
                X->s = -1;
                continue;
            }

            MBEDTLS_MPI_CHK( mpi_get_digit( &d, radix, s[i] ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_mul_int( &T, X, radix ) );

            if( X->s == 1 )
            {
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_int( X, &T, d ) );
            }
            else
            {
                MBEDTLS_MPI_CHK( mbedtls_mpi_sub_int( X, &T, d ) );
            }
        }
    }

cleanup:

    mbedtls_mpi_free( &T );

    return( ret );
}

/*
 * Helper to write the digits high-order first.
 */
static int mpi_write_hlp( mbedtls_mpi *X, int radix,
                          char **p, const size_t buflen )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi_uint r;
    size_t length = 0;
    char *p_end = *p + buflen;

    do
    {
        if( length >= buflen )
        {
            return( MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL );
        }

        MBEDTLS_MPI_CHK( mbedtls_mpi_mod_int( &r, X, radix ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_div_int( X, NULL, X, radix ) );
        /*
         * Write the residue in the current position, as an ASCII character.
         */
        if( r < 0xA )
            *(--p_end) = (char)( '0' + r );
        else
            *(--p_end) = (char)( 'A' + ( r - 0xA ) );

        length++;
    } while( mbedtls_mpi_cmp_int( X, 0 ) != 0 );

    memmove( *p, p_end, length );
    *p += length;

cleanup:

    return( ret );
}

/*
 * Export into an ASCII string
 */
int mbedtls_mpi_write_string( const mbedtls_mpi *X, int radix,
                              char *buf, size_t buflen, size_t *olen )
{
    int ret = 0;
    size_t n;
    char *p;
    mbedtls_mpi T;
    MPI_VALIDATE_RET( X    != NULL );
    MPI_VALIDATE_RET( olen != NULL );
    MPI_VALIDATE_RET( buflen == 0 || buf != NULL );

    if( radix < 2 || radix > 16 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    n = mbedtls_mpi_bitlen( X ); /* Number of bits necessary to present `n`. */
    if( radix >=  4 ) n >>= 1;   /* Number of 4-adic digits necessary to present
                                  * `n`. If radix > 4, this might be a strict
                                  * overapproximation of the number of
                                  * radix-adic digits needed to present `n`. */
    if( radix >= 16 ) n >>= 1;   /* Number of hexadecimal digits necessary to
                                  * present `n`. */

    n += 1; /* Terminating null byte */
    n += 1; /* Compensate for the divisions above, which round down `n`
             * in case it's not even. */
    n += 1; /* Potential '-'-sign. */
    n += ( n & 1 ); /* Make n even to have enough space for hexadecimal writing,
                     * which always uses an even number of hex-digits. */

    if( buflen < n )
    {
        *olen = n;
        return( MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL );
    }

    p = buf;
    mbedtls_mpi_init( &T );

    if( X->s == -1 )
    {
        *p++ = '-';
        buflen--;
    }

    if( radix == 16 )
    {
        int c;
        size_t i, j, k;

        for( i = X->n, k = 0; i > 0; i-- )
        {
            for( j = ciL; j > 0; j-- )
            {
                c = ( X->p[i - 1] >> ( ( j - 1 ) << 3) ) & 0xFF;

                if( c == 0 && k == 0 && ( i + j ) != 2 )
                    continue;

                *(p++) = "0123456789ABCDEF" [c / 16];
                *(p++) = "0123456789ABCDEF" [c % 16];
                k = 1;
            }
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &T, X ) );

        if( T.s == -1 )
            T.s = 1;

        MBEDTLS_MPI_CHK( mpi_write_hlp( &T, radix, &p, buflen ) );
    }

    *p++ = '\0';
    *olen = p - buf;

cleanup:

    mbedtls_mpi_free( &T );

    return( ret );
}

#if defined(MBEDTLS_FS_IO)
/*
 * Read X from an opened file
 */
int mbedtls_mpi_read_file( mbedtls_mpi *X, int radix, FILE *fin )
{
    mbedtls_mpi_uint d;
    size_t slen;
    char *p;
    /*
     * Buffer should have space for (short) label and decimal formatted MPI,
     * newline characters and '\0'
     */
    char s[ MBEDTLS_MPI_RW_BUFFER_SIZE ];

    MPI_VALIDATE_RET( X   != NULL );
    MPI_VALIDATE_RET( fin != NULL );

    if( radix < 2 || radix > 16 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    memset( s, 0, sizeof( s ) );
    if( fgets( s, sizeof( s ) - 1, fin ) == NULL )
        return( MBEDTLS_ERR_MPI_FILE_IO_ERROR );

    slen = strlen( s );
    if( slen == sizeof( s ) - 2 )
        return( MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL );

    if( slen > 0 && s[slen - 1] == '\n' ) { slen--; s[slen] = '\0'; }
    if( slen > 0 && s[slen - 1] == '\r' ) { slen--; s[slen] = '\0'; }

    p = s + slen;
    while( p-- > s )
        if( mpi_get_digit( &d, radix, *p ) != 0 )
            break;

    return( mbedtls_mpi_read_string( X, radix, p + 1 ) );
}

/*
 * Write X into an opened file (or stdout if fout == NULL)
 */
int mbedtls_mpi_write_file( const char *p, const mbedtls_mpi *X, int radix, FILE *fout )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t n, slen, plen;
    /*
     * Buffer should have space for (short) label and decimal formatted MPI,
     * newline characters and '\0'
     */
    char s[ MBEDTLS_MPI_RW_BUFFER_SIZE ];
    MPI_VALIDATE_RET( X != NULL );

    if( radix < 2 || radix > 16 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    memset( s, 0, sizeof( s ) );

    MBEDTLS_MPI_CHK( mbedtls_mpi_write_string( X, radix, s, sizeof( s ) - 2, &n ) );

    if( p == NULL ) p = "";

    plen = strlen( p );
    slen = strlen( s );
    s[slen++] = '\r';
    s[slen++] = '\n';

    if( fout != NULL )
    {
        if( fwrite( p, 1, plen, fout ) != plen ||
            fwrite( s, 1, slen, fout ) != slen )
            return( MBEDTLS_ERR_MPI_FILE_IO_ERROR );
    }
    else
        mbedtls_printf( "%s%s", p, s );

cleanup:

    return( ret );
}
#endif /* MBEDTLS_FS_IO */


/* Convert a big-endian byte array aligned to the size of mbedtls_mpi_uint
 * into the storage form used by mbedtls_mpi. */

static mbedtls_mpi_uint mpi_uint_bigendian_to_host_c( mbedtls_mpi_uint x )
{
    uint8_t i;
    unsigned char *x_ptr;
    mbedtls_mpi_uint tmp = 0;

    for( i = 0, x_ptr = (unsigned char*) &x; i < ciL; i++, x_ptr++ )
    {
        tmp <<= CHAR_BIT;
        tmp |= (mbedtls_mpi_uint) *x_ptr;
    }

    return( tmp );
}

static mbedtls_mpi_uint mpi_uint_bigendian_to_host( mbedtls_mpi_uint x )
{
#if defined(__BYTE_ORDER__)

/* Nothing to do on bigendian systems. */
#if ( __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )
    return( x );
#endif /* __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ */

#if ( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ )

/* For GCC and Clang, have builtins for byte swapping. */
#if defined(__GNUC__) && defined(__GNUC_PREREQ)
#if __GNUC_PREREQ(4,3)
#define have_bswap
#endif
#endif

#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_bswap32)  &&                 \
    __has_builtin(__builtin_bswap64)
#define have_bswap
#endif
#endif

#if defined(have_bswap)
    /* The compiler is hopefully able to statically evaluate this! */
    switch( sizeof(mbedtls_mpi_uint) )
    {
        case 4:
            return( __builtin_bswap32(x) );
        case 8:
            return( __builtin_bswap64(x) );
    }
#endif
#endif /* __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ */
#endif /* __BYTE_ORDER__ */

    /* Fall back to C-based reordering if we don't know the byte order
     * or we couldn't use a compiler-specific builtin. */
    return( mpi_uint_bigendian_to_host_c( x ) );
}

static void mpi_bigendian_to_host( mbedtls_mpi_uint * const p, size_t limbs )
{
    mbedtls_mpi_uint *cur_limb_left;
    mbedtls_mpi_uint *cur_limb_right;
    if( limbs == 0 )
        return;

    /*
     * Traverse limbs and
     * - adapt byte-order in each limb
     * - swap the limbs themselves.
     * For that, simultaneously traverse the limbs from left to right
     * and from right to left, as long as the left index is not bigger
     * than the right index (it's not a problem if limbs is odd and the
     * indices coincide in the last iteration).
     */
    for( cur_limb_left = p, cur_limb_right = p + ( limbs - 1 );
         cur_limb_left <= cur_limb_right;
         cur_limb_left++, cur_limb_right-- )
    {
        mbedtls_mpi_uint tmp;
        /* Note that if cur_limb_left == cur_limb_right,
         * this code effectively swaps the bytes only once. */
        tmp             = mpi_uint_bigendian_to_host( *cur_limb_left  );
        *cur_limb_left  = mpi_uint_bigendian_to_host( *cur_limb_right );
        *cur_limb_right = tmp;
    }
}

/*
 * Import X from unsigned binary data, little endian
 */
int mbedtls_mpi_read_binary_le( mbedtls_mpi *X,
                                const unsigned char *buf, size_t buflen )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t i;
    size_t const limbs = CHARS_TO_LIMBS( buflen );

    /* Ensure that target MPI has exactly the necessary number of limbs */
    if( X->n != limbs )
    {
        mbedtls_mpi_free( X );
        mbedtls_mpi_init( X );
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, limbs ) );
    }

    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

    for( i = 0; i < buflen; i++ )
        X->p[i / ciL] |= ((mbedtls_mpi_uint) buf[i]) << ((i % ciL) << 3);

cleanup:

    /*
     * This function is also used to import keys. However, wiping the buffers
     * upon failure is not necessary because failure only can happen before any
     * input is copied.
     */
    return( ret );
}

/*
 * Import X from unsigned binary data, big endian
 */
int mbedtls_mpi_read_binary( mbedtls_mpi *X, const unsigned char *buf, size_t buflen )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t const limbs    = CHARS_TO_LIMBS( buflen );
    size_t const overhead = ( limbs * ciL ) - buflen;
    unsigned char *Xp;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( buflen == 0 || buf != NULL );

    /* Ensure that target MPI has exactly the necessary number of limbs */
    if( X->n != limbs )
    {
        mbedtls_mpi_free( X );
        mbedtls_mpi_init( X );
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, limbs ) );
    }
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

    /* Avoid calling `memcpy` with NULL source argument,
     * even if buflen is 0. */
    if( buf != NULL )
    {
        Xp = (unsigned char*) X->p;
        memcpy( Xp + overhead, buf, buflen );

        mpi_bigendian_to_host( X->p, limbs );
    }

cleanup:

    /*
     * This function is also used to import keys. However, wiping the buffers
     * upon failure is not necessary because failure only can happen before any
     * input is copied.
     */
    return( ret );
}

/*
 * Export X into unsigned binary data, little endian
 */
int mbedtls_mpi_write_binary_le( const mbedtls_mpi *X,
                                 unsigned char *buf, size_t buflen )
{
    size_t stored_bytes = X->n * ciL;
    size_t bytes_to_copy;
    size_t i;

    if( stored_bytes < buflen )
    {
        bytes_to_copy = stored_bytes;
    }
    else
    {
        bytes_to_copy = buflen;

        /* The output buffer is smaller than the allocated size of X.
         * However X may fit if its leading bytes are zero. */
        for( i = bytes_to_copy; i < stored_bytes; i++ )
        {
            if( GET_BYTE( X, i ) != 0 )
                return( MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL );
        }
    }

    for( i = 0; i < bytes_to_copy; i++ )
        buf[i] = GET_BYTE( X, i );

    if( stored_bytes < buflen )
    {
        /* Write trailing 0 bytes */
        memset( buf + stored_bytes, 0, buflen - stored_bytes );
    }

    return( 0 );
}

/*
 * Export X into unsigned binary data, big endian
 */
int mbedtls_mpi_write_binary( const mbedtls_mpi *X,
                              unsigned char *buf, size_t buflen )
{
    size_t stored_bytes;
    size_t bytes_to_copy;
    unsigned char *p;
    size_t i;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( buflen == 0 || buf != NULL );

    stored_bytes = X->n * ciL;

    if( stored_bytes < buflen )
    {
        /* There is enough space in the output buffer. Write initial
         * null bytes and record the position at which to start
         * writing the significant bytes. In this case, the execution
         * trace of this function does not depend on the value of the
         * number. */
        bytes_to_copy = stored_bytes;
        p = buf + buflen - stored_bytes;
        memset( buf, 0, buflen - stored_bytes );
    }
    else
    {
        /* The output buffer is smaller than the allocated size of X.
         * However X may fit if its leading bytes are zero. */
        bytes_to_copy = buflen;
        p = buf;
        for( i = bytes_to_copy; i < stored_bytes; i++ )
        {
            if( GET_BYTE( X, i ) != 0 )
                return( MBEDTLS_ERR_MPI_BUFFER_TOO_SMALL );
        }
    }

    for( i = 0; i < bytes_to_copy; i++ )
        p[bytes_to_copy - i - 1] = GET_BYTE( X, i );

    return( 0 );
}

/*
 * Left-shift: X <<= count
 */
/*
int mbedtls_mpi_shift_l( mbedtls_mpi *X, size_t count )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t i, v0, t1;
    mbedtls_mpi_uint r0 = 0, r1;
    MPI_VALIDATE_RET( X != NULL );

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", X);
#endif

    v0 = count / (biL    );
    t1 = count & (biL - 1);

    i = mbedtls_mpi_bitlen( X ) + count;

    if( X->n * biL < i )
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, BITS_TO_LIMBS( i ) ) );


    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    ret = 0;
    
    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)X->p, 0, X->n);
        bignum_opB32 ((const uint32_t*)&count, 0, 1);
    #else
        bignum_opA ((const uint8_t*)X->p, 0, X->n*4);
        bignum_opB ((const uint8_t*)&count, 0, 4);
    #endif  

    bignum_configure(BIGNUM_MODE_SHIFT_L);
    bignum_start ();

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
            //TODO : mpi_grow 
            bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
            //TODO : mpi_grow 
            bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    mbedtls_printf("B     : %d\n", count);
    bignum_print("A << B", X);
    mbedtls_printf("rtn   : %d\n", ret);
#endif
    return( ret );
}
*/
int mbedtls_mpi_shift_l( mbedtls_mpi *X, size_t count )
{
    int ret;
    size_t i, v0, t1;
    mbedtls_mpi_uint r0 = 0, r1;
    MPI_VALIDATE_RET( X != NULL );

#ifdef BIGNUM_SHOW_RESULT
    // bignum_print("A", X);
#endif

    v0 = count / (biL    );
    t1 = count & (biL - 1);

    i = mbedtls_mpi_bitlen( X ) + count;

    if( X->n * biL < i )
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, BITS_TO_LIMBS( i ) ) );

    ret = 0;

    /*
     * shift by count / limb_size
     */
    if( v0 > 0 )
    {
        for( i = X->n; i > v0; i-- )
            X->p[i - 1] = X->p[i - v0 - 1];

        for( ; i > 0; i-- )
            X->p[i - 1] = 0;
    }

    /*
     * shift by count % limb_size
     */
    if( t1 > 0 )
    {
        for( i = v0; i < X->n; i++ )
        {
            r1 = X->p[i] >> (biL - t1);
            X->p[i] <<= t1;
            X->p[i] |= r0;
            r0 = r1;
        }
    }

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    // mbedtls_printf("B     : %d\n", count);
    // bignum_print("A << B", X);
    // mbedtls_printf("rtn   : %d\n", ret);
#endif
    return( ret );
}

/*
 * Right-shift: X >>= count
 */
/*
int mbedtls_mpi_shift_r( mbedtls_mpi *X, size_t count )
{
    size_t i, v0, v1, ret = 0;
    mbedtls_mpi_uint r0 = 0, r1;
    MPI_VALIDATE_RET( X != NULL );

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", X);
#endif

    v0 = count /  biL;
    v1 = count & (biL - 1);

    if( v0 > X->n || ( v0 == X->n && v1 > 0 ) )
    {
        ret = mbedtls_mpi_lset( X, 0 );
    #ifdef BIGNUM_SHOW_RESULT
        mbedtls_printf("rtn_R : %d\n", ret);
    #endif
        return ret;
    }

    
    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)X->p, 0, X->n);
        bignum_opB32 ((const uint32_t*)&count, 0, 1);
    #else
        bignum_opA ((const uint8_t*)X->p, 0, X->n*4);
        bignum_opB ((const uint8_t*)&count, 0, 4);
    #endif  

    bignum_configure(BIGNUM_MODE_SHIFT_R);
    bignum_start ();

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
            //TODO : mpi_shrink
            bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
            //TODO : mpi_shrink
            bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }
#ifdef BIGNUM_SHOW_RESULT
    mbedtls_printf("B : %d\n", count);
    bignum_print("A >> B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif
    return( 0 );
}
*/
int mbedtls_mpi_shift_r( mbedtls_mpi *X, size_t count )
{
    size_t i, v0, v1, ret = 0;
    mbedtls_mpi_uint r0 = 0, r1;
    MPI_VALIDATE_RET( X != NULL );

#ifdef BIGNUM_SHOW_RESULT
    // bignum_print("A", X);
#endif

    v0 = count /  biL;
    v1 = count & (biL - 1);

    if( v0 > X->n || ( v0 == X->n && v1 > 0 ) )
    {
        ret = mbedtls_mpi_lset( X, 0 );
    #ifdef BIGNUM_SHOW_RESULT
        // mbedtls_printf("rtn_R : %d\n", ret);
    #endif
        return ret;
    }

    /*
     * shift by count / limb_size
     */
    if( v0 > 0 )
    {
        for( i = 0; i < X->n - v0; i++ )
            X->p[i] = X->p[i + v0];

        for( ; i < X->n; i++ )
            X->p[i] = 0;
    }

    /*
     * shift by count % limb_size
     */
    if( v1 > 0 )
    {
        for( i = X->n; i > 0; i-- )
        {
            r1 = X->p[i - 1] << (biL - v1);
            X->p[i - 1] >>= v1;
            X->p[i - 1] |= r0;
            r0 = r1;
        }
    }
#ifdef BIGNUM_SHOW_RESULT
    // mbedtls_printf("B     : %d\n", count);
    // bignum_print("A >> B", X);
    // mbedtls_printf("rtn   : %d\n", ret);
#endif
    return( 0 );
}

/*
 * Compare unsigned values
 */
int mbedtls_mpi_cmp_abs( const mbedtls_mpi *X, const mbedtls_mpi *Y )
{
    size_t i, j;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );

    for( i = X->n; i > 0; i-- )
        if( X->p[i - 1] != 0 )
            break;

    for( j = Y->n; j > 0; j-- )
        if( Y->p[j - 1] != 0 )
            break;

    if( i == 0 && j == 0 )
        return( 0 );

    if( i > j ) return(  1 );
    if( j > i ) return( -1 );

    for( ; i > 0; i-- )
    {
        if( X->p[i - 1] > Y->p[i - 1] ) return(  1 );
        if( X->p[i - 1] < Y->p[i - 1] ) return( -1 );
    }

    return( 0 );
}

/*
 * Compare signed values
 */
int mbedtls_mpi_cmp_mpi( const mbedtls_mpi *X, const mbedtls_mpi *Y )
{
    size_t i, j;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );

    for( i = X->n; i > 0; i-- )
        if( X->p[i - 1] != 0 )
            break;

    for( j = Y->n; j > 0; j-- )
        if( Y->p[j - 1] != 0 )
            break;

    if( i == 0 && j == 0 )
        return( 0 );

    if( i > j ) return(  X->s );
    if( j > i ) return( -Y->s );

    if( X->s > 0 && Y->s < 0 ) return(  1 );
    if( Y->s > 0 && X->s < 0 ) return( -1 );

    for( ; i > 0; i-- )
    {
        if( X->p[i - 1] > Y->p[i - 1] ) return(  X->s );
        if( X->p[i - 1] < Y->p[i - 1] ) return( -X->s );
    }

    return( 0 );
}

/** Decide if an integer is less than the other, without branches.
 *
 * \param x         First integer.
 * \param y         Second integer.
 *
 * \return          1 if \p x is less than \p y, 0 otherwise
 */
static unsigned ct_lt_mpi_uint( const mbedtls_mpi_uint x,
        const mbedtls_mpi_uint y )
{
    mbedtls_mpi_uint ret;
    mbedtls_mpi_uint cond;

    /*
     * Check if the most significant bits (MSB) of the operands are different.
     */
    cond = ( x ^ y );
    /*
     * If the MSB are the same then the difference x-y will be negative (and
     * have its MSB set to 1 during conversion to unsigned) if and only if x<y.
     */
    ret = ( x - y ) & ~cond;
    /*
     * If the MSB are different, then the operand with the MSB of 1 is the
     * bigger. (That is if y has MSB of 1, then x<y is true and it is false if
     * the MSB of y is 0.)
     */
    ret |= y & cond;


    ret = ret >> ( biL - 1 );

    return (unsigned) ret;
}

/*
 * Compare signed values in constant time
 */
int mbedtls_mpi_lt_mpi_ct( const mbedtls_mpi *X, const mbedtls_mpi *Y,
        unsigned *ret )
{
    size_t i;
    /* The value of any of these variables is either 0 or 1 at all times. */
    unsigned cond, done, X_is_negative, Y_is_negative;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( Y != NULL );
    MPI_VALIDATE_RET( ret != NULL );

    if( X->n != Y->n )
        return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;

    /*
     * Set sign_N to 1 if N >= 0, 0 if N < 0.
     * We know that N->s == 1 if N >= 0 and N->s == -1 if N < 0.
     */
    X_is_negative = ( X->s & 2 ) >> 1;
    Y_is_negative = ( Y->s & 2 ) >> 1;

    /*
     * If the signs are different, then the positive operand is the bigger.
     * That is if X is negative (X_is_negative == 1), then X < Y is true and it
     * is false if X is positive (X_is_negative == 0).
     */
    cond = ( X_is_negative ^ Y_is_negative );
    *ret = cond & X_is_negative;

    /*
     * This is a constant-time function. We might have the result, but we still
     * need to go through the loop. Record if we have the result already.
     */
    done = cond;

    for( i = X->n; i > 0; i-- )
    {
        /*
         * If Y->p[i - 1] < X->p[i - 1] then X < Y is true if and only if both
         * X and Y are negative.
         *
         * Again even if we can make a decision, we just mark the result and
         * the fact that we are done and continue looping.
         */
        cond = ct_lt_mpi_uint( Y->p[i - 1], X->p[i - 1] );
        *ret |= cond & ( 1 - done ) & X_is_negative;
        done |= cond;

        /*
         * If X->p[i - 1] < Y->p[i - 1] then X < Y is true if and only if both
         * X and Y are positive.
         *
         * Again even if we can make a decision, we just mark the result and
         * the fact that we are done and continue looping.
         */
        cond = ct_lt_mpi_uint( X->p[i - 1], Y->p[i - 1] );
        *ret |= cond & ( 1 - done ) & ( 1 - X_is_negative );
        done |= cond;
    }

    return( 0 );
}

/*
 * Compare signed values
 */
int mbedtls_mpi_cmp_int( const mbedtls_mpi *X, mbedtls_mpi_sint z )
{
    mbedtls_mpi Y;
    mbedtls_mpi_uint p[1];
    MPI_VALIDATE_RET( X != NULL );

    *p  = ( z < 0 ) ? -z : z;
    Y.s = ( z < 0 ) ? -1 : 1;
    Y.n = 1;
    Y.p = p;

    return( mbedtls_mpi_cmp_mpi( X, &Y ) );
}


int bignum_equal(mbedtls_mpi *X, mbedtls_mpi *Y)
{
    int i;
    
    if(X->s != Y->s || X->n != Y->n)
        return 1;
    
    for(i=0; i<X->n; i++)
    {
        if(X->p[i] != Y->p[i])
        {
            return 1;
        }
    }

    return 0;
}

uint32_t bignum_get_real_length(const mbedtls_mpi *X)
{
    int j;
    for( j = (X->n-1); j > 0; j-- )
        if( X->p[j] != 0 )
            return j+1;
    return 0;
}
/*
 ret : 
    value    : first bigger index 
    positive : X > Y
    negative : Y < X
*/
int bignum_cmp_idx(const mbedtls_mpi *X, const mbedtls_mpi *Y)
{
    int i,j, s = 0;
    i = bignum_get_real_length(X);
    j = bignum_get_real_length(Y);

    if(i > j)
        return i;
    else if(i < j)
        return -j;
    
    for(;i > 0; i--)    
    {
        s = (X->p[i] > Y->p[i]);

        if(X->p[i] != Y->p[i])
        {
            return s * i;
        }
    }
    return 0;
}

/*
 * Unsigned addition: X = |A| + |B|  (HAC 14.7)
 */
int mbedtls_mpi_add_abs( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t i, j;
    mbedtls_mpi_uint *o, *p, c, tmp;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    if( X == B )
    {
        const mbedtls_mpi *T = A; A = X; B = T;
    }

    if( X != A )
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( X, A ) );

    /*
     * X should always be positive as a result of unsigned additions.
     */
    X->s = 1;

    for( j = B->n; j > 0; j-- )
        if( B->p[j - 1] != 0 )
            break;

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, j ) );

    o = B->p; p = X->p; c = 0;

    /*
     * tmp is used because it might happen that p == o
     */
    for( i = 0; i < j; i++, o++, p++ )
    {
        tmp= *o;
        *p +=  c; c  = ( *p <  c );
        *p += tmp; c += ( *p < tmp );
    }

    while( c != 0 )
    {
        if( i >= X->n )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, i + 1 ) );
            p = X->p + i;
        }

        *p += c; c = ( *p < c ); i++; p++;
    }

cleanup:

    return( ret );
}
/*
    This code is for using hw function but performance is not better then sw function
*/
/*
int mbedtls_mpi_add_abs( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret;
    size_t i, j;
    mbedtls_mpi_uint *o, *p, c, tmp = 0;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    X->s = 1;

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n*4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n*4);
    #endif  

    bignum_configure(BIGNUM_MODE_ADD);
    bignum_start();

    for( j = B->n; j > 0; j-- )
        if( B->p[j - 1] != 0 )
            break;

    for( i = A->n; i > 0; i-- )
        if( A->p[i - 1] != 0 )
            break;

    if(i == j)
    {
        tmp = ((A->p[i-1] + B->p[j-1]));
        if(tmp < A->p[i-1])
        {
            // mbedtls_printf("TMP : %08lx, A : %08lx, B : %08lx, i : %d, j : %d\n", tmp, A->p[i-1], B->p[j-1], i, j);
            mbedtls_mpi_shrink( X, i + 1 );
        }
        else
        {
            mbedtls_mpi_shrink( X, i );   
        }
    }
    else
    {
        mbedtls_mpi_shrink( X, (i > j) ? i : j );
    }

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }
    else
    {
        A("[func: %s] Fail to read buffer\n", __func__);
    }

    return( 0 );
}
*/

/**
 * Helper for mbedtls_mpi subtraction.
 *
 * Calculate d - s where d and s have the same size.
 * This function operates modulo (2^ciL)^n and returns the carry
 * (1 if there was a wraparound, i.e. if `d < s`, and 0 otherwise).
 *
 * \param n             Number of limbs of \p d and \p s.
 * \param[in,out] d     On input, the left operand.
 *                      On output, the result of the subtraction:
 * \param[in] s         The right operand.
 *
 * \return              1 if `d < s`.
 *                      0 if `d >= s`.
 */
static mbedtls_mpi_uint mpi_sub_hlp( size_t n,
                                     mbedtls_mpi_uint *d,
                                     const mbedtls_mpi_uint *s )
{
    size_t i;
    mbedtls_mpi_uint c, z;

    for( i = c = 0; i < n; i++, s++, d++ )
    {
        z = ( *d <  c );     *d -=  c;
        c = ( *d < *s ) + z; *d -= *s;
    }

    return( c );
}

/*
 * Unsigned subtraction: X = |A| - |B|  (HAC 14.9, 14.10)
 */
int mbedtls_mpi_sub_abs( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    mbedtls_mpi TB;
    int ret = 0;
    size_t n;
    mbedtls_mpi_uint carry;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    mbedtls_mpi_init( &TB );

    if( X == B )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TB, B ) );
        B = &TB;
    }

    if( X != A )
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( X, A ) );

    /*
     * X should always be positive as a result of unsigned subtractions.
     */
    X->s = 1;

    ret = 0;

    for( n = B->n; n > 0; n-- )
        if( B->p[n - 1] != 0 )
            break;

    carry = mpi_sub_hlp( n, X->p, B->p );
    if( carry != 0 )
    {
        /* Propagate the carry to the first nonzero limb of X. */
        for( ; n < X->n && X->p[n] == 0; n++ )
            --X->p[n];
        /* If we ran out of space for the carry, it means that the result
         * is negative. */
        if( n == X->n )
            return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );
        --X->p[n];
    }

cleanup:

    mbedtls_mpi_free( &TB );

    return( ret );
}
/*
int mbedtls_mpi_sub_abs( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    mbedtls_mpi TB;
    int ret = 0;
    size_t n;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    if( mbedtls_mpi_cmp_abs( A, B ) < 0 )
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );

    mbedtls_mpi_init( &TB );

    if( X == B )
    {
        mbedtls_mpi_copy( &TB, B );
        B = &TB;
    }

    if( X != A )
        mbedtls_mpi_copy( X, A );

    X->s = 1;

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );
    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n*4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n*4);
    #endif  
        bignum_configure(BIGNUM_MODE_SUB);
        bignum_start();

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        //TODO : mpi_shrink
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        //TODO : mpi_shrink
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }

    mbedtls_mpi_free( &TB );

    return( ret );
}
*/
/*
 * Signed addition: X = A + B
 */
int mbedtls_mpi_add_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret, s;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    s = A->s;
    if( A->s * B->s < 0 )
    {
        if( mbedtls_mpi_cmp_abs( A, B ) >= 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, A, B ) );
            X->s =  s;
        }
        else
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, B, A ) );
            X->s = -s;
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_add_abs( X, A, B ) );
        X->s = s;
    }

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", A);
    bignum_print("B", B);
    bignum_print("A + B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif
    return( ret );
}

/*
 * Signed subtraction: X = A - B
 */
int mbedtls_mpi_sub_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret, s;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    s = A->s;
    if( A->s * B->s > 0 )
    {
        if( mbedtls_mpi_cmp_abs( A, B ) >= 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, A, B ) );
            X->s =  s;
        }
        else
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, B, A ) );
            X->s = -s;
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_add_abs( X, A, B ) );
        X->s = s;
    }

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", A);
    bignum_print("B", B);
    bignum_print("A - B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif
    return( ret );
}
/*
int mbedtls_mpi_sub_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret, s;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    s = A->s;
    if( A->s * B->s > 0 )
    {
        if( mbedtls_mpi_cmp_abs( A, B ) >= 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, A, B ) );
            X->s =  s;
        }
        else
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( X, B, A ) );
            X->s = -s;
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_add_abs( X, A, B ) );
        X->s = s;
    }

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", A);
    bignum_print("B", B);
    bignum_print("A - B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif
    return( ret );
}
*/
/*
 * Signed addition: X = A + b
 */
int mbedtls_mpi_add_int( mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_sint b )
{
    mbedtls_mpi B;
    mbedtls_mpi_uint p[1];
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );

    p[0] = ( b < 0 ) ? -b : b;
    B.s = ( b < 0 ) ? -1 : 1;
    B.n = 1;
    B.p = p;

    return( mbedtls_mpi_add_mpi( X, A, &B ) );
}

/*
 * Signed subtraction: X = A - b
 */
int mbedtls_mpi_sub_int( mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_sint b )
{
    mbedtls_mpi B;
    mbedtls_mpi_uint p[1];
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );

    p[0] = ( b < 0 ) ? -b : b;
    B.s = ( b < 0 ) ? -1 : 1;
    B.n = 1;
    B.p = p;

    return( mbedtls_mpi_sub_mpi( X, A, &B ) );
}

/*
 * Helper for mbedtls_mpi multiplication
 */
static
#if defined(__APPLE__) && defined(__arm__)
/*
 * Apple LLVM version 4.2 (clang-425.0.24) (based on LLVM 3.2svn)
 * appears to need this to prevent bad ARM code generation at -O3.
 */
__attribute__ ((noinline))
#endif
void mpi_mul_hlp( size_t i, mbedtls_mpi_uint *s, mbedtls_mpi_uint *d, mbedtls_mpi_uint b )
{
    mbedtls_mpi_uint c = 0, t = 0;

#if defined(MULADDC_HUIT)
    for( ; i >= 8; i -= 8 )
    {
        MULADDC_INIT
        MULADDC_HUIT
        MULADDC_STOP
    }

    for( ; i > 0; i-- )
    {
        MULADDC_INIT
        MULADDC_CORE
        MULADDC_STOP
    }
#else /* MULADDC_HUIT */
    for( ; i >= 16; i -= 16 )
    {
        MULADDC_INIT
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE

        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_STOP
    }

    for( ; i >= 8; i -= 8 )
    {
        MULADDC_INIT
        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE

        MULADDC_CORE   MULADDC_CORE
        MULADDC_CORE   MULADDC_CORE
        MULADDC_STOP
    }

    for( ; i > 0; i-- )
    {
        MULADDC_INIT
        MULADDC_CORE
        MULADDC_STOP
    }
#endif /* MULADDC_HUIT */

    t++;

    do {
        *d += c; c = ( *d < c ); d++;
    }
    while( c != 0 );
}

/*
 * Baseline multiplication: X = A * B  (HAC 14.12)
 */
#if 0
int mbedtls_mpi_mul_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = 0;
    size_t i, j;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    for( i = A->n; i > 0; i-- )
        if( A->p[i - 1] != 0 )
            break;

    for( j = B->n; j > 0; j-- )
        if( B->p[j - 1] != 0 )
            break;

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n * 4);
    #endif  

    bignum_configure(BIGNUM_MODE_MUL);
    bignum_start ();

    mbedtls_mpi_grow( X, i + j );
    mbedtls_mpi_lset( X, 0 );

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    } else {
        return -1;
    }

    X->s = A->s * B->s;

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", A);
    bignum_print("B", B);
    bignum_print("A * B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif

    return( ret );
}
#else
int mbedtls_mpi_mul_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = 0;
    size_t i, j;
    mbedtls_mpi TA, TB;

#ifdef CHECK_MAX_BIT_LEN
    if(mbedtls_mpi_cmp_abs(A, &max_bit_len_mul_op) > 0)
        mbedtls_mpi_copy(&max_bit_len_mul_op, A);
    if(mbedtls_mpi_cmp_abs(B, &max_bit_len_mul_op) > 0)
        mbedtls_mpi_copy(&max_bit_len_mul_op, B);
#endif

    mbedtls_mpi_init( &TA ); mbedtls_mpi_init( &TB );

    if( X == A ) { MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TA, A ) ); A = &TA; }
    if( X == B ) { MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TB, B ) ); B = &TB; }

    for( i = A->n; i > 0; i-- )
        if( A->p[i - 1] != 0 )
            break;

    for( j = B->n; j > 0; j-- )
        if( B->p[j - 1] != 0 )
            break;

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, i + j ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

    for( i++; j > 0; j-- )
        mpi_mul_hlp( i - 1, A->p, X->p + j - 1, B->p[j - 1] );

    X->s = A->s * B->s;

cleanup:
#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A", A);
    bignum_print("B", B);
    bignum_print("A * B", X);
    mbedtls_printf("rtn : %d\n", ret);
#endif
#ifdef CHECK_MAX_BIT_LEN
    if(mbedtls_mpi_cmp_abs(X, &max_bit_len_mul_res) > 0)
        mbedtls_mpi_copy(&max_bit_len_mul_res, X);
#endif
    mbedtls_mpi_free( &TB ); mbedtls_mpi_free( &TA );

	//if(A->n > 16 || B->n > 16)
		//A("A: %d B: %d, X: %d\n", A->n, B->n, X->n);

    return( ret );
}
#endif

/*
 * Baseline multiplication: X = A * b
 */
int mbedtls_mpi_mul_int( mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_uint b )
{
    mbedtls_mpi B;
    mbedtls_mpi_uint p[1];
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );

    B.s = 1;
    B.n = 1;
    B.p = p;
    p[0] = b;

    return( mbedtls_mpi_mul_mpi( X, A, &B ) );
}

/*
 * Unsigned integer divide - double mbedtls_mpi_uint dividend, u1/u0, and
 * mbedtls_mpi_uint divisor, d
 */
static mbedtls_mpi_uint mbedtls_int_div_int( mbedtls_mpi_uint u1,
            mbedtls_mpi_uint u0, mbedtls_mpi_uint d, mbedtls_mpi_uint *r )
{
#if defined(MBEDTLS_HAVE_UDBL)
    mbedtls_t_udbl dividend, quotient;
#else
    const mbedtls_mpi_uint radix = (mbedtls_mpi_uint) 1 << biH;
    const mbedtls_mpi_uint uint_halfword_mask = ( (mbedtls_mpi_uint) 1 << biH ) - 1;
    mbedtls_mpi_uint d0, d1, q0, q1, rAX, r0, quotient;
    mbedtls_mpi_uint u0_msw, u0_lsw;
    size_t s;
#endif

    /*
     * Check for overflow
     */
    if( 0 == d || u1 >= d )
    {
        if (r != NULL) *r = ~0;

        return ( ~0 );
    }

#if defined(MBEDTLS_HAVE_UDBL)
    dividend  = (mbedtls_t_udbl) u1 << biL;
    dividend |= (mbedtls_t_udbl) u0;
    quotient = dividend / d;
    if( quotient > ( (mbedtls_t_udbl) 1 << biL ) - 1 )
        quotient = ( (mbedtls_t_udbl) 1 << biL ) - 1;

    if( r != NULL )
        *r = (mbedtls_mpi_uint)( dividend - (quotient * d ) );

    return (mbedtls_mpi_uint) quotient;
#else

    /*
     * Algorithm D, Section 4.3.1 - The Art of Computer Programming
     *   Vol. 2 - Seminumerical Algorithms, Knuth
     */

    /*
     * Normalize the divisor, d, and dividend, u0, u1
     */
    s = mbedtls_clz( d );
    d = d << s;

    u1 = u1 << s;
    u1 |= ( u0 >> ( biL - s ) ) & ( -(mbedtls_mpi_sint)s >> ( biL - 1 ) );
    u0 =  u0 << s;

    d1 = d >> biH;
    d0 = d & uint_halfword_mask;

    u0_msw = u0 >> biH;
    u0_lsw = u0 & uint_halfword_mask;

    /*
     * Find the first quotient and remainder
     */
    q1 = u1 / d1;
    r0 = u1 - d1 * q1;

    while( q1 >= radix || ( q1 * d0 > radix * r0 + u0_msw ) )
    {
        q1 -= 1;
        r0 += d1;

        if ( r0 >= radix ) break;
    }

    rAX = ( u1 * radix ) + ( u0_msw - q1 * d );
    q0 = rAX / d1;
    r0 = rAX - q0 * d1;

    while( q0 >= radix || ( q0 * d0 > radix * r0 + u0_lsw ) )
    {
        q0 -= 1;
        r0 += d1;

        if ( r0 >= radix ) break;
    }

    if (r != NULL)
        *r = ( rAX * radix + u0_lsw - q0 * d ) >> s;

    quotient = q1 * radix + q0;

    return quotient;
#endif
}

/*
 * Division by mbedtls_mpi: A = Q * B + R  (HAC 14.20)
 */
int mbedtls_mpi_div_mpi( mbedtls_mpi *Q, mbedtls_mpi *R, const mbedtls_mpi *A,
                         const mbedtls_mpi *B )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t i, n, t, k;
    mbedtls_mpi X, Y, Z, T1, T2;
    mbedtls_mpi_uint TP2[3];
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    if( mbedtls_mpi_cmp_int( B, 0 ) == 0 )
        return( MBEDTLS_ERR_MPI_DIVISION_BY_ZERO );

    mbedtls_mpi_init( &X ); mbedtls_mpi_init( &Y ); mbedtls_mpi_init( &Z );
    mbedtls_mpi_init( &T1 );
    /*
     * Avoid dynamic memory allocations for constant-size T2.
     *
     * T2 is used for comparison only and the 3 limbs are assigned explicitly,
     * so nobody increase the size of the MPI and we're safe to use an on-stack
     * buffer.
     */
    T2.s = 1;
    T2.n = sizeof( TP2 ) / sizeof( *TP2 );
    T2.p = TP2;

    if( mbedtls_mpi_cmp_abs( A, B ) < 0 )
    {
        if( Q != NULL ) MBEDTLS_MPI_CHK( mbedtls_mpi_lset( Q, 0 ) );
        if( R != NULL ) MBEDTLS_MPI_CHK( mbedtls_mpi_copy( R, A ) );
        return( 0 );
    }

    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &X, A ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &Y, B ) );
    X.s = Y.s = 1;

    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &Z, A->n + 2 ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &Z,  0 ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &T1, 2 ) );

    k = mbedtls_mpi_bitlen( &Y ) % biL;
    if( k < biL - 1 )
    {
        k = biL - 1 - k;
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &X, k ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &Y, k ) );
    }
    else k = 0;

    n = X.n - 1;
    t = Y.n - 1;
    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &Y, biL * ( n - t ) ) );

    while( mbedtls_mpi_cmp_mpi( &X, &Y ) >= 0 )
    {
        Z.p[n - t]++;
        MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &X, &X, &Y ) );
    }
    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &Y, biL * ( n - t ) ) );

    for( i = n; i > t ; i-- )
    {
        if( X.p[i] >= Y.p[t] )
            Z.p[i - t - 1] = ~0;
        else
        {
            Z.p[i - t - 1] = mbedtls_int_div_int( X.p[i], X.p[i - 1],
                                                            Y.p[t], NULL);
        }

        T2.p[0] = ( i < 2 ) ? 0 : X.p[i - 2];
        T2.p[1] = ( i < 1 ) ? 0 : X.p[i - 1];
        T2.p[2] = X.p[i];

        Z.p[i - t - 1]++;
        do
        {
            Z.p[i - t - 1]--;

            MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &T1, 0 ) );
            T1.p[0] = ( t < 1 ) ? 0 : Y.p[t - 1];
            T1.p[1] = Y.p[t];
            MBEDTLS_MPI_CHK( mbedtls_mpi_mul_int( &T1, &T1, Z.p[i - t - 1] ) );
        }
        while( mbedtls_mpi_cmp_mpi( &T1, &T2 ) > 0 );

        MBEDTLS_MPI_CHK( mbedtls_mpi_mul_int( &T1, &Y, Z.p[i - t - 1] ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &T1,  biL * ( i - t - 1 ) ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &X, &X, &T1 ) );

        if( mbedtls_mpi_cmp_int( &X, 0 ) < 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &T1, &Y ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &T1, biL * ( i - t - 1 ) ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( &X, &X, &T1 ) );
            Z.p[i - t - 1]--;
        }
    }

    if( Q != NULL )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( Q, &Z ) );
        Q->s = A->s * B->s;
    }

    if( R != NULL )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &X, k ) );
        X.s = A->s;
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( R, &X ) );

        if( mbedtls_mpi_cmp_int( R, 0 ) == 0 )
            R->s = 1;
    }

cleanup:

    mbedtls_mpi_free( &X ); mbedtls_mpi_free( &Y ); mbedtls_mpi_free( &Z );
    mbedtls_mpi_free( &T1 );
    mbedtls_platform_zeroize( TP2, sizeof( TP2 ) );

    return( ret );
}

/*
 * Division by int: A = Q * b + R
 */
int mbedtls_mpi_div_int( mbedtls_mpi *Q, mbedtls_mpi *R,
                         const mbedtls_mpi *A,
                         mbedtls_mpi_sint b )
{
    mbedtls_mpi B;
    mbedtls_mpi_uint p[1];
    MPI_VALIDATE_RET( A != NULL );

    p[0] = ( b < 0 ) ? -b : b;
    B.s = ( b < 0 ) ? -1 : 1;
    B.n = 1;
    B.p = p;

    return( mbedtls_mpi_div_mpi( Q, R, A, &B ) );
}

/*
 * Modulo: R = A mod B
 */
int mbedtls_mpi_mod_mpi_sw( mbedtls_mpi *R, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = 0;

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("MOD REAL A", A);
    bignum_print("MOD REAL B", B);
#endif
#ifdef CHECK_MAX_BIT_LEN
    if(mbedtls_mpi_cmp_abs(A, &max_bit_len_mod_op) > 0)
        mbedtls_mpi_copy(&max_bit_len_mod_op, A);
    if(mbedtls_mpi_cmp_abs(B, &max_bit_len_mod_op) > 0)
        mbedtls_mpi_copy(&max_bit_len_mod_op, B);
#endif

    if( mbedtls_mpi_cmp_int( B, 0 ) < 0 )
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );

    MBEDTLS_MPI_CHK( mbedtls_mpi_div_mpi( NULL, R, A, B ) );

    while( mbedtls_mpi_cmp_int( R, 0 ) < 0 )
      MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( R, R, B ) );

    while( mbedtls_mpi_cmp_mpi( R, B ) >= 0 )
      MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( R, R, B ) );


cleanup:
#ifdef BIGNUM_SHOW_RESULT
    // bignum_print("A", A);
    // bignum_print("B", B);
    bignum_print("A MOD B", R);
    mbedtls_printf("rtn : %d\n", ret);
#endif
#ifdef CHECK_MAX_BIT_LEN
    if(mbedtls_mpi_cmp_abs(R, &max_bit_len_mod_res) > 0)
        mbedtls_mpi_copy(&max_bit_len_mod_res, R);
#endif

#if 0
	if(A->n > 16 || B->n > 16)
		A("A2: %d B2: %d R:%d\n", A->n, B->n, R->n);
#endif

	return( ret );
}

/*
 * Modulo: R = A mod B
 */
int mbedtls_mpi_mod_mpi( mbedtls_mpi *R, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = 0;
    int length, sig;
    size_t i, j;

    MPI_VALIDATE_RET( R != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("MOD REAL A", A);
    bignum_print("MOD REAL B", B);
#endif

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        if(A->n > 16)
            bignum_opB32 ((const uint32_t*)(A->p + 16), 0, (A->n-16));
        bignum_opN32 ((const uint32_t*)B->p, 0, B->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
        bignum_opN ((const uint8_t*)B->p, 0, B->n * 4);
    #endif  

    bignum_configure(BIGNUM_MODE_MOD);
    bignum_start ();

    if( mbedtls_mpi_cmp_int( B, 0 ) < 0 )
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );

    i = bignum_get_real_length(A);
    j = bignum_get_real_length(B);
    if (i > 16 || j > 16) {
        return mbedtls_mpi_mod_mpi_sw(R, A, B);
    }

    sig =  A->s * B->s;
    length = (i > j) ? j : i;

    if( mbedtls_mpi_cmp_abs( A, B ) >= 0 )
    {
        length = bignum_get_real_length(B);
    }
    else
    {
        length = bignum_get_real_length(A);
    }

    mbedtls_mpi_shrink(R, length);
    mbedtls_mpi_lset(R, 0);

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)R->p, R->n);
    #else
        bignum_get_result ((uint8_t*)R->p, R->n*4);
    #endif  
    } else {
        return -1;
    }

    if(sig < 0)
    {
        R->s = 1;
        mbedtls_mpi_sub_mpi(R, B, R);
        R->s = B->s;
    }

#ifdef BIGNUM_SHOW_RESULT
    bignum_print("A MOD B", R);
    mbedtls_printf("rtn : %d\n", ret);
#endif

    return( ret );
}

/*
 * Modulo: X = A * B mod C
 */
int mbedtls_mpi_mul_mpi_mod_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *C )
{
    int length, sig;
    size_t i, j, k;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );
    MPI_VALIDATE_RET( C != NULL );

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
        bignum_opN32 ((const uint32_t*)C->p, 0, C->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n * 4);
        bignum_opN ((const uint8_t*)C->p, 0, C->n * 4);
    #endif  

    bignum_configure(BIGNUM_MODE_MULMOD);
    bignum_start ();

    if( mbedtls_mpi_cmp_int( C, 0 ) < 0 )
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );

    //Get length for calculate return size
    i = bignum_get_real_length(A);
    j = bignum_get_real_length(B);
    k = bignum_get_real_length(C);
    if (i > 16 || j > 16 || k > 16) {
        if (mbedtls_mpi_mul_mpi(X, A, B) != 0)
        {
            return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );
        }
        return mbedtls_mpi_mod_mpi_sw(X, X, C);
    }

    sig =  A->s * B->s;
    length = ((i+j) > k) ? k : (i+j);

    mbedtls_mpi_shrink(X, length);
    mbedtls_mpi_lset(X, 0);

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    } else {
        return -1;
    }

    if(sig < 0)
    {
        X->s = 1;
        mbedtls_mpi_sub_abs(X, C, X);
        X->s = C->s;
    }

    return 0;
}

/*
 * Multiplication Modulo Ready: modulus N
 */
int mbedtls_mul_mod_ready( const mbedtls_mpi *N )
{
    if (bignum_get_real_length(N) > 16) {
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );
    }

    if(mbedtls_mpi_cmp_int(N, 0) < 0) {
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );
    }

    bignum_clear_op( BIGNUM_OPN_CLR | BIGNUM_OPA_CLR | BIGNUM_OPB_CLR );
#ifdef WORD_OPERATION
    bignum_opN32 ((const uint32_t*)N->p, 0, N->n);
#else
    bignum_opN ((const uint8_t*)N->p, 0, N->n * 4);
#endif
    bignum_configure(BIGNUM_MODE_MULMOD);

    return 0;
}

/*
 * Multiplication Modulo Start: A * B mod N
 */
int mbedtls_mul_mod_start( const mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *N )
{
    int i, j;

    if (A->s * B->s < 0) {
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );
    }

    i = bignum_get_real_length(A);
    j = bignum_get_real_length(B);
    if (i > 16 || j > 16) {
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );
    }

    bignum_clear_op( BIGNUM_OPA_CLR | BIGNUM_OPB_CLR );
#ifdef WORD_OPERATION
    bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
    bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
#else
    bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
    bignum_opB ((const uint8_t*)B->p, 0, B->n * 4);
#endif
    bignum_start();

    return 0;
}

/*
 * Multiplication Modulo Done: X = A * B mod N
 */
int mbedtls_mul_mod_done( mbedtls_mpi *X, const mbedtls_mpi *N )
{
    mbedtls_mpi_shrink(X, N->n);
    mbedtls_mpi_lset(X, 0);
    if (bignum_is_done()) {
#ifdef WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
#else
        bignum_get_result ((uint8_t*)X->p, X->n * 4);
#endif
    } else {
        return -1;
    }

    return 0;
}

/*
 * Multiplication Modulo after Ready: X = A * B mod N
 */
int mbedtls_mul_mod( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *N )
{
    return mbedtls_mul_mod_start( A, B, N ) | mbedtls_mul_mod_done( X, N );
}

/*
 * Scalar exponentiation: X = A^b mod C
 */
int mbedtls_mpi_exp_int_mod_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const uint32_t b, const mbedtls_mpi *C )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    int sig;
    mbedtls_mpi T;
    uint32_t d = 0, bit = 1;

    if (b == 0) {
        return mbedtls_mpi_lset(X, 1);
    }

    sig = (b & 0x1) ? A->s : 1;
    if (sig < 0 && X == A) {
        return -1;
    }

    mbedtls_mpi_shrink(X, bignum_get_real_length(C));
    if (X != A) {
        mbedtls_mpi_copy( X, A );
    }

    if (b == 1) {
        return mbedtls_mpi_mod_mpi( X, X, C );
    }

    mbedtls_mpi_init( &T );
    mbedtls_mpi_lset( &T, 1 );

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opN32 ((const uint32_t*)C->p, 0, C->n);
    #else
        bignum_opN ((const uint8_t*)C->p, 0, C->n * 4);
    #endif

    bignum_configure(BIGNUM_MODE_MULMOD);

    while (b > d) {
        if (!((b ^ d) & bit)) {
            #ifdef  WORD_OPERATION
                bignum_opA32 ((const uint32_t*)X->p, 0, X->n);
                bignum_opB32 ((const uint32_t*)X->p, 0, X->n);
            #else
                bignum_opA ((const uint8_t*)X->p, 0, X->n * 4);
                bignum_opB ((const uint8_t*)X->p, 0, X->n * 4);
            #endif
        } else if (mbedtls_mpi_cmp_int( &T, 1 ) == 0) {
            mbedtls_mpi_copy( &T, X );
            d |= bit;
            continue;
        } else {
            #ifdef  WORD_OPERATION
                bignum_opA32 ((const uint32_t*)X->p, 0, X->n);
                bignum_opB32 ((const uint32_t*)T.p, 0, T.n);
            #else
                bignum_opA ((const uint8_t*)X->p, 0, X->n * 4);
                bignum_opB ((const uint8_t*)T.p, 0, T.n * 4);
            #endif
        }

        bignum_start();
        if (!bignum_is_done()) {
            goto cleanup;
        }

        if (!((b ^ d) & bit)) {
            #ifdef  WORD_OPERATION
                bignum_get32_result ((uint32_t*)X->p, X->n);
            #else
                bignum_get_result ((uint8_t*)X->p, X->n*4);
            #endif
            bit <<= 1;
        } else {
            #ifdef  WORD_OPERATION
                bignum_get32_result ((uint32_t*)T.p, T.n);
            #else
                bignum_get_result ((uint8_t*)T.p, T.n*4);
            #endif
            d |= bit;
        }
    }

    mbedtls_mpi_copy( X, &T );

    if (sig < 0) {
        X->s = 1;
        mbedtls_mpi_sub_abs(X, C, X);
        X->s = C->s;
    }
    ret = 0;

cleanup:
    mbedtls_mpi_free(&T);

    return ret;
}

/*
 * Modulo: X = A - B mod C
 */
int mbedtls_mpi_sub_mpi_mod_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *C )
{
    int length;
    int k;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );
    MPI_VALIDATE_RET( C != NULL );

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
        bignum_opN32 ((const uint32_t*)C->p, 0, C->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n * 4);
        bignum_opN ((const uint8_t*)C->p, 0, C->n * 4);
    #endif  

    bignum_configure(BIGNUM_MODE_SUBMOD);
    bignum_start ();

    k = bignum_cmp_idx(A, B);
    
    if(k >= 0)
    {
        length = k;
    }
    else
    {
        length = C->n;
    }
    
    mbedtls_mpi_lset(X, 0);
    mbedtls_mpi_shrink( X, length);

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }

    if(k < 0)
    {
        X->s = 1;
        mbedtls_mpi_sub_abs(X, C, X);
        X->s = C->s;
    }

    return 0;
}

/*
 * Modulo: X = A + B mod C
 */
int mbedtls_mpi_add_mpi_mod_mpi( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *C )
{
    int length;
    int i,j,k;
    mbedtls_mpi_uint tmp;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );
    MPI_VALIDATE_RET( C != NULL );

    bignum_clear_op( BIGNUM_OPN_CLR |
                     BIGNUM_OPA_CLR |
                     BIGNUM_OPB_CLR );

    #ifdef  WORD_OPERATION
        bignum_opA32 ((const uint32_t*)A->p, 0, A->n);
        bignum_opB32 ((const uint32_t*)B->p, 0, B->n);
        bignum_opN32 ((const uint32_t*)C->p, 0, C->n);
    #else
        bignum_opA ((const uint8_t*)A->p, 0, A->n * 4);
        bignum_opB ((const uint8_t*)B->p, 0, B->n * 4);
        bignum_opN ((const uint8_t*)C->p, 0, C->n * 4);
    #endif  

    bignum_configure(BIGNUM_MODE_ADDMOD);
    bignum_start ();

    i = bignum_get_real_length(A);
    j = bignum_get_real_length(B);
    
    if(i == j)
    {
        tmp = ((A->p[i-1] + B->p[j-1]));
        if(tmp < A->p[i-1])
        {
            length = i + 1;
        }
        else
        {
            length = i;
        }
    }
    else
    {
        length = (i > j) ? i : j;
    }

    length = (length > C->n) ? C->n : length;
    
    mbedtls_mpi_lset(X, 0);
    mbedtls_mpi_shrink( X, length);

    if(bignum_is_done()) {
    #ifdef  WORD_OPERATION
        bignum_get32_result ((uint32_t*)X->p, X->n);
    #else
        bignum_get_result ((uint8_t*)X->p, X->n*4);
    #endif  
    }

    // if(s < 0)
    // {
    //     X->s = 1;
    //     mbedtls_mpi_sub_abs(X, C, X);
    //     X->s = C->s;
    // }

    return 0;
}

/*
 * Modulo: r = A mod b
 */
int mbedtls_mpi_mod_int( mbedtls_mpi_uint *r, const mbedtls_mpi *A, mbedtls_mpi_sint b )
{
    size_t i;
    mbedtls_mpi_uint x, y, z;
    MPI_VALIDATE_RET( r != NULL );
    MPI_VALIDATE_RET( A != NULL );

    if( b == 0 )
        return( MBEDTLS_ERR_MPI_DIVISION_BY_ZERO );

    if( b < 0 )
        return( MBEDTLS_ERR_MPI_NEGATIVE_VALUE );

    /*
     * handle trivial cases
     */
    if( b == 1 )
    {
        *r = 0;
        return( 0 );
    }

    if( b == 2 )
    {
        *r = A->p[0] & 1;
        return( 0 );
    }

    /*
     * general case
     */
    for( i = A->n, y = 0; i > 0; i-- )
    {
        x  = A->p[i - 1];
        y  = ( y << biH ) | ( x >> biH );
        z  = y / b;
        y -= z * b;

        x <<= biH;
        y  = ( y << biH ) | ( x >> biH );
        z  = y / b;
        y -= z * b;
    }

    /*
     * If A is negative, then the current y represents a negative value.
     * Flipping it to the positive side.
     */
    if( A->s < 0 && y != 0 )
        y = b - y;

    *r = y;

    return( 0 );
}

/*
 * Fast Montgomery initialization (thanks to Tom St Denis)
 */
static void mpi_montg_init( mbedtls_mpi_uint *mm, const mbedtls_mpi *N )
{
    mbedtls_mpi_uint x, m0 = N->p[0];
    unsigned int i;

    x  = m0;
    x += ( ( m0 + 2 ) & 4 ) << 1;

    for( i = biL; i >= 8; i /= 2 )
        x *= ( 2 - ( m0 * x ) );

    *mm = ~x + 1;
}

/** Montgomery multiplication: A = A * B * R^-1 mod N  (HAC 14.36)
 *
 * \param[in,out]   A   One of the numbers to multiply.
 *                      It must have at least as many limbs as N
 *                      (A->n >= N->n), and any limbs beyond n are ignored.
 *                      On successful completion, A contains the result of
 *                      the multiplication A * B * R^-1 mod N where
 *                      R = (2^ciL)^n.
 * \param[in]       B   One of the numbers to multiply.
 *                      It must be nonzero and must not have more limbs than N
 *                      (B->n <= N->n).
 * \param[in]       N   The modulo. N must be odd.
 * \param           mm  The value calculated by `mpi_montg_init(&mm, N)`.
 *                      This is -N^-1 mod 2^ciL.
 * \param[in,out]   T   A bignum for temporary storage.
 *                      It must be at least twice the limb size of N plus 2
 *                      (T->n >= 2 * (N->n + 1)).
 *                      Its initial content is unused and
 *                      its final content is indeterminate.
 *                      Note that unlike the usual convention in the library
 *                      for `const mbedtls_mpi*`, the content of T can change.
 */
static void mpi_montmul( mbedtls_mpi *A, const mbedtls_mpi *B, const mbedtls_mpi *N, mbedtls_mpi_uint mm,
                         const mbedtls_mpi *T )
{
    size_t i, n, m;
    mbedtls_mpi_uint u0, u1, *d;

    memset( T->p, 0, T->n * ciL );

    d = T->p;
    n = N->n;
    m = ( B->n < n ) ? B->n : n;

    for( i = 0; i < n; i++ )
    {
        /*
         * T = (T + u0*B + u1*N) / 2^biL
         */
        u0 = A->p[i];
        u1 = ( d[0] + u0 * B->p[0] ) * mm;

        mpi_mul_hlp( m, B->p, d, u0 );
        mpi_mul_hlp( n, N->p, d, u1 );

        *d++ = u0; d[n + 1] = 0;
    }

    /* At this point, d is either the desired result or the desired result
     * plus N. We now potentially subtract N, avoiding leaking whether the
     * subtraction is performed through side channels. */

    /* Copy the n least significant limbs of d to A, so that
     * A = d if d < N (recall that N has n limbs). */
    memcpy( A->p, d, n * ciL );
    /* If d >= N then we want to set A to d - N. To prevent timing attacks,
     * do the calculation without using conditional tests. */
    /* Set d to d0 + (2^biL)^n - N where d0 is the current value of d. */
    d[n] += 1;
    d[n] -= mpi_sub_hlp( n, d, N->p );
    /* If d0 < N then d < (2^biL)^n
     * so d[n] == 0 and we want to keep A as it is.
     * If d0 >= N then d >= (2^biL)^n, and d <= (2^biL)^n + N < 2 * (2^biL)^n
     * so d[n] == 1 and we want to set A to the result of the subtraction
     * which is d - (2^biL)^n, i.e. the n least significant limbs of d.
     * This exactly corresponds to a conditional assignment. */
    mpi_safe_cond_assign( n, A->p, d, (unsigned char) d[n] );
}

/*
 * Montgomery reduction: A = A * R^-1 mod N
 *
 * See mpi_montmul() regarding constraints and guarantees on the parameters.
 */
static void mpi_montred( mbedtls_mpi *A, const mbedtls_mpi *N,
                         mbedtls_mpi_uint mm, const mbedtls_mpi *T )
{
    mbedtls_mpi_uint z = 1;
    mbedtls_mpi U;

    U.n = U.s = (int) z;
    U.p = &z;

    mpi_montmul( A, &U, N, mm, T );
}

/*
 * Sliding-window exponentiation: X = A^E mod N  (HAC 14.85)
 */
int mbedtls_mpi_exp_mod( mbedtls_mpi *X, const mbedtls_mpi *A,
                         const mbedtls_mpi *E, const mbedtls_mpi *N,
                         mbedtls_mpi *_RR )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t wbits, wsize, one = 1;
    size_t i, j, nblimbs;
    size_t bufsize, nbits;
    mbedtls_mpi_uint ei, mm, state;
    mbedtls_mpi RR, T, W[ 2 << MBEDTLS_MPI_WINDOW_SIZE ], Apos;
    int neg;

    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( E != NULL );
    MPI_VALIDATE_RET( N != NULL );

    if( mbedtls_mpi_cmp_int( N, 0 ) <= 0 || ( N->p[0] & 1 ) == 0 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    if( mbedtls_mpi_cmp_int( E, 0 ) < 0 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    /*
     * Init temps and window size
     */
    mpi_montg_init( &mm, N );
    mbedtls_mpi_init( &RR ); mbedtls_mpi_init( &T );
    mbedtls_mpi_init( &Apos );
    memset( W, 0, sizeof( W ) );

    i = mbedtls_mpi_bitlen( E );

    wsize = ( i > 671 ) ? 6 : ( i > 239 ) ? 5 :
            ( i >  79 ) ? 4 : ( i >  23 ) ? 3 : 1;

#if( MBEDTLS_MPI_WINDOW_SIZE < 6 )
    if( wsize > MBEDTLS_MPI_WINDOW_SIZE )
        wsize = MBEDTLS_MPI_WINDOW_SIZE;
#endif

    j = N->n + 1;
    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, j ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &W[1],  j ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &T, j * 2 ) );

    /*
     * Compensate for negative A (and correct at the end)
     */
    neg = ( A->s == -1 );
    if( neg )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &Apos, A ) );
        Apos.s = 1;
        A = &Apos;
    }

    /*
     * If 1st call, pre-compute R^2 mod N
     */
    if( _RR == NULL || _RR->p == NULL )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &RR, 1 ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &RR, N->n * 2 * biL ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_mod_mpi( &RR, &RR, N ) );

        if( _RR != NULL )
            memcpy( _RR, &RR, sizeof( mbedtls_mpi ) );
    }
    else
        memcpy( &RR, _RR, sizeof( mbedtls_mpi ) );

    /*
     * W[1] = A * R^2 * R^-1 mod N = A * R mod N
     */
    if( mbedtls_mpi_cmp_mpi( A, N ) >= 0 )
        MBEDTLS_MPI_CHK( mbedtls_mpi_mod_mpi( &W[1], A, N ) );
    else
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &W[1], A ) );

    mpi_montmul( &W[1], &RR, N, mm, &T );

    /*
     * X = R^2 * R^-1 mod N = R mod N
     */
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( X, &RR ) );
    mpi_montred( X, N, mm, &T );

    if( wsize > 1 )
    {
        /*
         * W[1 << (wsize - 1)] = W[1] ^ (wsize - 1)
         */
        j =  one << ( wsize - 1 );

        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &W[j], N->n + 1 ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &W[j], &W[1]    ) );

        for( i = 0; i < wsize - 1; i++ )
            mpi_montmul( &W[j], &W[j], N, mm, &T );

        /*
         * W[i] = W[i - 1] * W[1]
         */
        for( i = j + 1; i < ( one << wsize ); i++ )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_grow( &W[i], N->n + 1 ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &W[i], &W[i - 1] ) );

            mpi_montmul( &W[i], &W[1], N, mm, &T );
        }
    }

    nblimbs = E->n;
    bufsize = 0;
    nbits   = 0;
    wbits   = 0;
    state   = 0;

    while( 1 )
    {
        if( bufsize == 0 )
        {
            if( nblimbs == 0 )
                break;

            nblimbs--;

            bufsize = sizeof( mbedtls_mpi_uint ) << 3;
        }

        bufsize--;

        ei = (E->p[nblimbs] >> bufsize) & 1;

        /*
         * skip leading 0s
         */
        if( ei == 0 && state == 0 )
            continue;

        if( ei == 0 && state == 1 )
        {
            /*
             * out of window, square X
             */
            mpi_montmul( X, X, N, mm, &T );
            continue;
        }

        /*
         * add ei to current window
         */
        state = 2;

        nbits++;
        wbits |= ( ei << ( wsize - nbits ) );

        if( nbits == wsize )
        {
            /*
             * X = X^wsize R^-1 mod N
             */
            for( i = 0; i < wsize; i++ )
                mpi_montmul( X, X, N, mm, &T );

            /*
             * X = X * W[wbits] R^-1 mod N
             */
            mpi_montmul( X, &W[wbits], N, mm, &T );

            state--;
            nbits = 0;
            wbits = 0;
        }
    }

    /*
     * process the remaining bits
     */
    for( i = 0; i < nbits; i++ )
    {
        mpi_montmul( X, X, N, mm, &T );

        wbits <<= 1;

        if( ( wbits & ( one << wsize ) ) != 0 )
            mpi_montmul( X, &W[1], N, mm, &T );
    }

    /*
     * X = A^E * R * R^-1 mod N = A^E mod N
     */
    mpi_montred( X, N, mm, &T );

    if( neg && E->n != 0 && ( E->p[0] & 1 ) != 0 )
    {
        X->s = -1;
        MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( X, N, X ) );
    }

cleanup:

    for( i = ( one << ( wsize - 1 ) ); i < ( one << wsize ); i++ )
        mbedtls_mpi_free( &W[i] );

    mbedtls_mpi_free( &W[1] ); mbedtls_mpi_free( &T ); mbedtls_mpi_free( &Apos );

    if( _RR == NULL || _RR->p == NULL )
        mbedtls_mpi_free( &RR );

    return( ret );
}

/*
 * Greatest common divisor: G = gcd(A, B)  (HAC 14.54)
 */
int mbedtls_mpi_gcd( mbedtls_mpi *G, const mbedtls_mpi *A, const mbedtls_mpi *B )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t lz, lzt;
    mbedtls_mpi TA, TB;

    MPI_VALIDATE_RET( G != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( B != NULL );

    mbedtls_mpi_init( &TA ); mbedtls_mpi_init( &TB );

    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TA, A ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TB, B ) );

    lz = mbedtls_mpi_lsb( &TA );
    lzt = mbedtls_mpi_lsb( &TB );

    if( lzt < lz )
        lz = lzt;

    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TA, lz ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TB, lz ) );

    TA.s = TB.s = 1;

    while( mbedtls_mpi_cmp_int( &TA, 0 ) != 0 )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TA, mbedtls_mpi_lsb( &TA ) ) );
        MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TB, mbedtls_mpi_lsb( &TB ) ) );

        if( mbedtls_mpi_cmp_mpi( &TA, &TB ) >= 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( &TA, &TA, &TB ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TA, 1 ) );
        }
        else
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_abs( &TB, &TB, &TA ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TB, 1 ) );
        }
    }

    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_l( &TB, lz ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( G, &TB ) );

cleanup:

    mbedtls_mpi_free( &TA ); mbedtls_mpi_free( &TB );

    return( ret );
}

/*
 * Fill X with size bytes of random.
 *
 * Use a temporary bytes representation to make sure the result is the same
 * regardless of the platform endianness (useful when f_rng is actually
 * deterministic, eg for tests).
 */
int mbedtls_mpi_fill_random( mbedtls_mpi *X, size_t size,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t const limbs = CHARS_TO_LIMBS( size );
    size_t const overhead = ( limbs * ciL ) - size;
    unsigned char *Xp;

    MPI_VALIDATE_RET( X     != NULL );
    MPI_VALIDATE_RET( f_rng != NULL );

    /* Ensure that target MPI has exactly the necessary number of limbs */
    if( X->n != limbs )
    {
        mbedtls_mpi_free( X );
        mbedtls_mpi_init( X );
        MBEDTLS_MPI_CHK( mbedtls_mpi_grow( X, limbs ) );
    }
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( X, 0 ) );

    Xp = (unsigned char*) X->p;
    f_rng( p_rng, Xp + overhead, size );

    mpi_bigendian_to_host( X->p, limbs );

cleanup:
    return( ret );
}

/*
 * Modular inverse: X = A^-1 mod N  (HAC 14.61 / 14.64)
 */
int mbedtls_mpi_inv_mod( mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *N )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi G, TA, TU, U1, U2, TB, TV, V1, V2;
    MPI_VALIDATE_RET( X != NULL );
    MPI_VALIDATE_RET( A != NULL );
    MPI_VALIDATE_RET( N != NULL );

    if( mbedtls_mpi_cmp_int( N, 1 ) <= 0 )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    mbedtls_mpi_init( &TA ); mbedtls_mpi_init( &TU ); mbedtls_mpi_init( &U1 ); mbedtls_mpi_init( &U2 );
    mbedtls_mpi_init( &G ); mbedtls_mpi_init( &TB ); mbedtls_mpi_init( &TV );
    mbedtls_mpi_init( &V1 ); mbedtls_mpi_init( &V2 );

    MBEDTLS_MPI_CHK( mbedtls_mpi_gcd( &G, A, N ) );

    if( mbedtls_mpi_cmp_int( &G, 1 ) != 0 )
    {
        ret = MBEDTLS_ERR_MPI_NOT_ACCEPTABLE;
        goto cleanup;
    }

    MBEDTLS_MPI_CHK( mbedtls_mpi_mod_mpi( &TA, A, N ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TU, &TA ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TB, N ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &TV, N ) );

    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &U1, 1 ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &U2, 0 ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &V1, 0 ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_lset( &V2, 1 ) );

    do
    {
        while( ( TU.p[0] & 1 ) == 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TU, 1 ) );

            if( ( U1.p[0] & 1 ) != 0 || ( U2.p[0] & 1 ) != 0 )
            {
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( &U1, &U1, &TB ) );
                MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &U2, &U2, &TA ) );
            }

            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &U1, 1 ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &U2, 1 ) );
        }

        while( ( TV.p[0] & 1 ) == 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &TV, 1 ) );

            if( ( V1.p[0] & 1 ) != 0 || ( V2.p[0] & 1 ) != 0 )
            {
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( &V1, &V1, &TB ) );
                MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &V2, &V2, &TA ) );
            }

            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &V1, 1 ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &V2, 1 ) );
        }

        if( mbedtls_mpi_cmp_mpi( &TU, &TV ) >= 0 )
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &TU, &TU, &TV ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &U1, &U1, &V1 ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &U2, &U2, &V2 ) );
        }
        else
        {
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &TV, &TV, &TU ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &V1, &V1, &U1 ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &V2, &V2, &U2 ) );
        }
    }
    while( mbedtls_mpi_cmp_int( &TU, 0 ) != 0 );

    while( mbedtls_mpi_cmp_int( &V1, 0 ) < 0 )
        MBEDTLS_MPI_CHK( mbedtls_mpi_add_mpi( &V1, &V1, N ) );

    while( mbedtls_mpi_cmp_mpi( &V1, N ) >= 0 )
        MBEDTLS_MPI_CHK( mbedtls_mpi_sub_mpi( &V1, &V1, N ) );

    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( X, &V1 ) );

cleanup:

    mbedtls_mpi_free( &TA ); mbedtls_mpi_free( &TU ); mbedtls_mpi_free( &U1 ); mbedtls_mpi_free( &U2 );
    mbedtls_mpi_free( &G ); mbedtls_mpi_free( &TB ); mbedtls_mpi_free( &TV );
    mbedtls_mpi_free( &V1 ); mbedtls_mpi_free( &V2 );

    return( ret );
}

#if defined(MBEDTLS_GENPRIME)

static const int small_prime[] =
{
        3,    5,    7,   11,   13,   17,   19,   23,
       29,   31,   37,   41,   43,   47,   53,   59,
       61,   67,   71,   73,   79,   83,   89,   97,
      101,  103,  107,  109,  113,  127,  131,  137,
      139,  149,  151,  157,  163,  167,  173,  179,
      181,  191,  193,  197,  199,  211,  223,  227,
      229,  233,  239,  241,  251,  257,  263,  269,
      271,  277,  281,  283,  293,  307,  311,  313,
      317,  331,  337,  347,  349,  353,  359,  367,
      373,  379,  383,  389,  397,  401,  409,  419,
      421,  431,  433,  439,  443,  449,  457,  461,
      463,  467,  479,  487,  491,  499,  503,  509,
      521,  523,  541,  547,  557,  563,  569,  571,
      577,  587,  593,  599,  601,  607,  613,  617,
      619,  631,  641,  643,  647,  653,  659,  661,
      673,  677,  683,  691,  701,  709,  719,  727,
      733,  739,  743,  751,  757,  761,  769,  773,
      787,  797,  809,  811,  821,  823,  827,  829,
      839,  853,  857,  859,  863,  877,  881,  883,
      887,  907,  911,  919,  929,  937,  941,  947,
      953,  967,  971,  977,  983,  991,  997, -103
};

/*
 * Small divisors test (X must be positive)
 *
 * Return values:
 * 0: no small factor (possible prime, more tests needed)
 * 1: certain prime
 * MBEDTLS_ERR_MPI_NOT_ACCEPTABLE: certain non-prime
 * other negative: error
 */
static int mpi_check_small_factors( const mbedtls_mpi *X )
{
    int ret = 0;
    size_t i;
    mbedtls_mpi_uint r;

    if( ( X->p[0] & 1 ) == 0 )
        return( MBEDTLS_ERR_MPI_NOT_ACCEPTABLE );

    for( i = 0; small_prime[i] > 0; i++ )
    {
        if( mbedtls_mpi_cmp_int( X, small_prime[i] ) <= 0 )
            return( 1 );

        MBEDTLS_MPI_CHK( mbedtls_mpi_mod_int( &r, X, small_prime[i] ) );

        if( r == 0 )
            return( MBEDTLS_ERR_MPI_NOT_ACCEPTABLE );
    }

cleanup:
    return( ret );
}

/*
 * Miller-Rabin pseudo-primality test  (HAC 4.24)
 */
static int mpi_miller_rabin( const mbedtls_mpi *X, size_t rounds,
                             int (*f_rng)(void *, unsigned char *, size_t),
                             void *p_rng )
{
    int ret, count;
    size_t i, j, k, s;
    mbedtls_mpi W, R, T, A, RR;

    MPI_VALIDATE_RET( X     != NULL );
    MPI_VALIDATE_RET( f_rng != NULL );

    mbedtls_mpi_init( &W ); mbedtls_mpi_init( &R );
    mbedtls_mpi_init( &T ); mbedtls_mpi_init( &A );
    mbedtls_mpi_init( &RR );

    /*
     * W = |X| - 1
     * R = W >> lsb( W )
     */
    MBEDTLS_MPI_CHK( mbedtls_mpi_sub_int( &W, X, 1 ) );
    s = mbedtls_mpi_lsb( &W );
    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &R, &W ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &R, s ) );

    for( i = 0; i < rounds; i++ )
    {
        /*
         * pick a random A, 1 < A < |X| - 1
         */
        count = 0;
        do {
            MBEDTLS_MPI_CHK( mbedtls_mpi_fill_random( &A, X->n * ciL, f_rng, p_rng ) );

            j = mbedtls_mpi_bitlen( &A );
            k = mbedtls_mpi_bitlen( &W );
            if (j > k) {
                A.p[A.n - 1] &= ( (mbedtls_mpi_uint) 1 << ( k - ( A.n - 1 ) * biL - 1 ) ) - 1;
            }

            if (count++ > 30) {
                ret = MBEDTLS_ERR_MPI_NOT_ACCEPTABLE;
                goto cleanup;
            }

        } while ( mbedtls_mpi_cmp_mpi( &A, &W ) >= 0 ||
                  mbedtls_mpi_cmp_int( &A, 1 )  <= 0    );

        /*
         * A = A^R mod |X|
         */
        MBEDTLS_MPI_CHK( mbedtls_mpi_exp_mod( &A, &A, &R, X, &RR ) );

        if( mbedtls_mpi_cmp_mpi( &A, &W ) == 0 ||
            mbedtls_mpi_cmp_int( &A,  1 ) == 0 )
            continue;

        j = 1;
        while( j < s && mbedtls_mpi_cmp_mpi( &A, &W ) != 0 )
        {
            /*
             * A = A * A mod |X|
             */
            MBEDTLS_MPI_CHK( mbedtls_mpi_mul_mpi( &T, &A, &A ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_mod_mpi( &A, &T, X  ) );

            if( mbedtls_mpi_cmp_int( &A, 1 ) == 0 )
                break;

            j++;
        }

        /*
         * not prime if A != |X| - 1 or A == 1
         */
        if( mbedtls_mpi_cmp_mpi( &A, &W ) != 0 ||
            mbedtls_mpi_cmp_int( &A,  1 ) == 0 )
        {
            ret = MBEDTLS_ERR_MPI_NOT_ACCEPTABLE;
            break;
        }
    }

cleanup:
    mbedtls_mpi_free( &W ); mbedtls_mpi_free( &R );
    mbedtls_mpi_free( &T ); mbedtls_mpi_free( &A );
    mbedtls_mpi_free( &RR );

    return( ret );
}

/*
 * Pseudo-primality test: small factors, then Miller-Rabin
 */
int mbedtls_mpi_is_prime_ext( const mbedtls_mpi *X, int rounds,
                              int (*f_rng)(void *, unsigned char *, size_t),
                              void *p_rng )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_mpi XX;
    MPI_VALIDATE_RET( X     != NULL );
    MPI_VALIDATE_RET( f_rng != NULL );

    XX.s = 1;
    XX.n = X->n;
    XX.p = X->p;

    if( mbedtls_mpi_cmp_int( &XX, 0 ) == 0 ||
        mbedtls_mpi_cmp_int( &XX, 1 ) == 0 )
        return( MBEDTLS_ERR_MPI_NOT_ACCEPTABLE );

    if( mbedtls_mpi_cmp_int( &XX, 2 ) == 0 )
        return( 0 );

    if( ( ret = mpi_check_small_factors( &XX ) ) != 0 )
    {
        if( ret == 1 )
            return( 0 );

        return( ret );
    }

    return( mpi_miller_rabin( &XX, rounds, f_rng, p_rng ) );
}

#if !defined(MBEDTLS_DEPRECATED_REMOVED)
/*
 * Pseudo-primality test, error probability 2^-80
 */
int mbedtls_mpi_is_prime( const mbedtls_mpi *X,
                  int (*f_rng)(void *, unsigned char *, size_t),
                  void *p_rng )
{
    MPI_VALIDATE_RET( X     != NULL );
    MPI_VALIDATE_RET( f_rng != NULL );

    /*
     * In the past our key generation aimed for an error rate of at most
     * 2^-80. Since this function is deprecated, aim for the same certainty
     * here as well.
     */
    return( mbedtls_mpi_is_prime_ext( X, 40, f_rng, p_rng ) );
}
#endif

/*
 * Prime number generation
 *
 * To generate an RSA key in a way recommended by FIPS 186-4, both primes must
 * be either 1024 bits or 1536 bits long, and flags must contain
 * MBEDTLS_MPI_GEN_PRIME_FLAG_LOW_ERR.
 */
int mbedtls_mpi_gen_prime( mbedtls_mpi *X, size_t nbits, int flags,
                   int (*f_rng)(void *, unsigned char *, size_t),
                   void *p_rng )
{
#ifdef MBEDTLS_HAVE_INT64
// ceil(2^63.5)
#define CEIL_MAXUINT_DIV_SQRT2 0xb504f333f9de6485ULL
#else
// ceil(2^31.5)
#define CEIL_MAXUINT_DIV_SQRT2 0xb504f334U
#endif
    int ret = MBEDTLS_ERR_MPI_NOT_ACCEPTABLE;
    size_t k, n;
    int rounds;
    mbedtls_mpi_uint r;
    mbedtls_mpi Y;

    MPI_VALIDATE_RET( X     != NULL );
    MPI_VALIDATE_RET( f_rng != NULL );

    if( nbits < 3 || nbits > MBEDTLS_MPI_MAX_BITS )
        return( MBEDTLS_ERR_MPI_BAD_INPUT_DATA );

    mbedtls_mpi_init( &Y );

    n = BITS_TO_LIMBS( nbits );

    if( ( flags & MBEDTLS_MPI_GEN_PRIME_FLAG_LOW_ERR ) == 0 )
    {
        /*
         * 2^-80 error probability, number of rounds chosen per HAC, table 4.4
         */
        rounds = ( ( nbits >= 1300 ) ?  2 : ( nbits >=  850 ) ?  3 :
                   ( nbits >=  650 ) ?  4 : ( nbits >=  350 ) ?  8 :
                   ( nbits >=  250 ) ? 12 : ( nbits >=  150 ) ? 18 : 27 );
    }
    else
    {
        /*
         * 2^-100 error probability, number of rounds computed based on HAC,
         * fact 4.48
         */
        rounds = ( ( nbits >= 1450 ) ?  4 : ( nbits >=  1150 ) ?  5 :
                   ( nbits >= 1000 ) ?  6 : ( nbits >=   850 ) ?  7 :
                   ( nbits >=  750 ) ?  8 : ( nbits >=   500 ) ? 13 :
                   ( nbits >=  250 ) ? 28 : ( nbits >=   150 ) ? 40 : 51 );
    }

    while( 1 )
    {
        MBEDTLS_MPI_CHK( mbedtls_mpi_fill_random( X, n * ciL, f_rng, p_rng ) );
        /* make sure generated number is at least (nbits-1)+0.5 bits (FIPS 186-4 §B.3.3 steps 4.4, 5.5) */
        if( X->p[n-1] < CEIL_MAXUINT_DIV_SQRT2 ) continue;

        k = n * biL;
        if( k > nbits ) MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( X, k - nbits ) );
        X->p[0] |= 1;

        if( ( flags & MBEDTLS_MPI_GEN_PRIME_FLAG_DH ) == 0 )
        {
            ret = mbedtls_mpi_is_prime_ext( X, rounds, f_rng, p_rng );

            if( ret != MBEDTLS_ERR_MPI_NOT_ACCEPTABLE )
                goto cleanup;
        }
        else
        {
            /*
             * An necessary condition for Y and X = 2Y + 1 to be prime
             * is X = 2 mod 3 (which is equivalent to Y = 2 mod 3).
             * Make sure it is satisfied, while keeping X = 3 mod 4
             */

            X->p[0] |= 2;

            MBEDTLS_MPI_CHK( mbedtls_mpi_mod_int( &r, X, 3 ) );
            if( r == 0 )
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_int( X, X, 8 ) );
            else if( r == 1 )
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_int( X, X, 4 ) );

            /* Set Y = (X-1) / 2, which is X / 2 because X is odd */
            MBEDTLS_MPI_CHK( mbedtls_mpi_copy( &Y, X ) );
            MBEDTLS_MPI_CHK( mbedtls_mpi_shift_r( &Y, 1 ) );

            while( 1 )
            {
                /*
                 * First, check small factors for X and Y
                 * before doing Miller-Rabin on any of them
                 */
                if( ( ret = mpi_check_small_factors(  X         ) ) == 0 &&
                    ( ret = mpi_check_small_factors( &Y         ) ) == 0 &&
                    ( ret = mpi_miller_rabin(  X, rounds, f_rng, p_rng  ) )
                                                                    == 0 &&
                    ( ret = mpi_miller_rabin( &Y, rounds, f_rng, p_rng  ) )
                                                                    == 0 )
                    goto cleanup;

                if( ret != MBEDTLS_ERR_MPI_NOT_ACCEPTABLE )
                    goto cleanup;

                /*
                 * Next candidates. We want to preserve Y = (X-1) / 2 and
                 * Y = 1 mod 2 and Y = 2 mod 3 (eq X = 3 mod 4 and X = 2 mod 3)
                 * so up Y by 6 and X by 12.
                 */
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_int(  X,  X, 12 ) );
                MBEDTLS_MPI_CHK( mbedtls_mpi_add_int( &Y, &Y, 6  ) );
            }
        }
    }

cleanup:

    mbedtls_mpi_free( &Y );

    return( ret );
}

#endif /* MBEDTLS_GENPRIME */

void bignum_print(char *tHeader, const mbedtls_mpi *R)
{
    int i;
    mbedtls_printf("\n%-8s : %c0x", tHeader, (R->s==-1?'-':'+'));
    //mbedtls_printf("%8s : %d 0x", tHeader, R->s);
    for(i=(R->n-1); i>=0; i--)
    {
        mbedtls_printf("%08lx", R->p[i]);
    }
    mbedtls_printf("\n");
}

#if defined(MBEDTLS_SELF_TEST)

/*
 * Checkup routine
 */
int mbedtls_mpi_self_test(cmd_tbl_t *t, int argc, char *argv[])
{
    int ret, i;
    mbedtls_mpi A, B, C, D, E, X;
    uint32_t B_T, A_T;

    #ifdef ASM_OPT
        A("ASM HW MODE\n");
    #else
        A("NORMAL HW MODE");
    #endif


    mbedtls_mpi_init( &A ); mbedtls_mpi_init( &B ); mbedtls_mpi_init( &C ); mbedtls_mpi_init( &X );
    mbedtls_mpi_init( &D ); mbedtls_mpi_init( &E );

    //1. Set Endian
    bignum_init(0);

    mbedtls_mpi_read_string( &A, 16,
        "00000001" );

    mbedtls_mpi_read_string( &B, 16,
        "00000001");

    mbedtls_mpi_read_string( &C, 16,
        "00000001");

    mbedtls_mpi_read_string( &D, 16,
        "bc2f168fe7d799ff07089dee3f34585c"
        "b004732d3410df41efff15c62ed56bed" );
    D.s = -1;
    
    mbedtls_mpi_read_string( &E, 16,
        "5ac635d8aa3a93e7b3ebbd55769886bc"
        "651d06b0cc53b0f63bce3c3e27d2604b" );

    bignum_print("A", &A);
    bignum_print("B", &B);
    bignum_print("C", &C);
    bignum_print("D", &D);
    bignum_print("E", &E);

    mbedtls_mpi_mul_mpi(&D, &D, &D);
    bignum_print("D = D * D", &D);

    mbedtls_mpi_read_string( &D, 16,
        "bc2f168fe7d799ff07089dee3f34585c"
        "b004732d3410df41efff15c62ed56bed" );
    D.s = -1;
    
    mbedtls_mpi_read_string( &E, 16,
        "5ac635d8aa3a93e7b3ebbd55769886bc"
        "651d06b0cc53b0f63bce3c3e27d2604b" );

    mbedtls_mpi_mul_mpi(&D, &D, &E);
    bignum_print("D = D * E", &D);

    mbedtls_mpi_read_string( &D, 16,
        "bc2f168fe7d799ff07089dee3f34585c"
        "b004732d3410df41efff15c62ed56bed" );
    D.s = -1;
    
    mbedtls_mpi_read_string( &E, 16,
        "5ac635d8aa3a93e7b3ebbd55769886bc"
        "651d06b0cc53b0f63bce3c3e27d2604b" );

    mbedtls_mpi_mul_mpi(&E, &D, &E);
    bignum_print("E = D * E", &E);
    



    mbedtls_mpi_free( &A );
    mbedtls_mpi_free( &B );
    mbedtls_mpi_free( &C );
    mbedtls_mpi_free( &D );
    mbedtls_mpi_free( &E );
    mbedtls_mpi_free( &X );

    // mbedtls_printf("Current Free Heap Size = %10d\n", xPortGetFreeHeapSize() );
    // mbedtls_printf("Minumum Free Heap Size = %10d\n", xPortGetMinimumEverFreeHeapSize() );

    return( 0 );
}

SUBCMD(test,
	  bignum,
	  mbedtls_mpi_self_test,
	  "test bignum for ECC encrypt",
	  "test bignum");

/*
 * Checkup routine
 */
int mbedtls_mpi_self_test_2(cmd_tbl_t *t, int argc, char *argv[])
{
    int ret, i;
    mbedtls_mpi A, B, C, D, E, X;
    uint32_t B_T, A_T;

    #ifdef ASM_OPT
        A("ASM HW MODE\n");
    #else
        A("NORMAL HW MODE");
    #endif


    mbedtls_mpi_init( &A ); mbedtls_mpi_init( &B ); mbedtls_mpi_init( &C ); mbedtls_mpi_init( &X );
    mbedtls_mpi_init( &D ); mbedtls_mpi_init( &E );

    bignum_init(0);

    mbedtls_mpi_read_string( &A, 16,
        "00000001" );

    mbedtls_mpi_read_string( &B, 16,
        "00000001");

    mbedtls_mpi_read_string( &C, 16,
        "00000001");

    mbedtls_mpi_read_string( &D, 16,
        "FFFFFFFFFF6f403c7a0e800000000000" \
        "fda56af29736f403c7a0e85d4ce0f20a" \
        "2e1980a35f2e1980a35f2e1980a35faa" \
        "AEffffffff6f403c7a0e800000000000" );
    
    mbedtls_mpi_read_string( &E, 16,
        "FBfffBEEFA0000000100000000000000" \
        "2e1980a35f2e1980a35f2e1980a35faa" \
        "fda56af29736f403c7a0e85d4ce0f20a" \
        "0xffffffff6f403c7a0e800000000000" );

    D.s = -1;
    

    bignum_print("A", &A);
    bignum_print("B", &B);
    bignum_print("C", &C);
    bignum_print("D", &D);
    bignum_print("E", &E);
    

    //A << 8
    mbedtls_mpi_shift_l(&A, 512);
    bignum_print("A << 512", &A);

    mbedtls_mpi_shift_l(&B, 256);
    bignum_print("B << 256", &B);

    // mbedtls_mpi_shift_l(&C, 511);
    // bignum_print("C << 511", &C);


    ////////////////////////////////////////////////////
    //TEST OP TIME
    ////////////////////////////////////////////////////
    //EXCEPT MEM ALLOC TIME
    mbedtls_mpi_add_mpi(&X, &D, &E);
    
    B_T = TSF;
    mbedtls_mpi_add_mpi(&X, &D, &E);
    A_T = TSF;
    bignum_print("D ADD E", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_add_mpi(&X, &E, &D);
    A_T = TSF;    
    bignum_print("E ADD D", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_sub_mpi(&X, &D, &E);
    A_T = TSF;
    bignum_print("D SUB E", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_sub_mpi(&X, &E, &D);
    A_T = TSF;
    bignum_print("E SUB D", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    //EXCEPT MEM ALLOC TIME
    mbedtls_mpi_mul_mpi(&X, &D, &E);

    B_T = TSF;
    mbedtls_mpi_mul_mpi(&X, &D, &E);
    A_T = TSF;
    bignum_print("D * E", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_mul_mpi(&X, &E, &D);
    A_T = TSF;
    bignum_print("E * D", &X);
    A("OP TIME : %d \n\n", A_T - B_T);
    
    B_T = TSF;
    mbedtls_mpi_mod_mpi(&X, &D, &E);
    A_T = TSF;
    bignum_print("D MOD E", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_mod_mpi(&X, &E, &D);
    A_T = TSF;
    bignum_print("E MOD D", &X);
    A("OP TIME : %d \n\n", A_T - B_T);

    //EXCEPT MEM ALLOC TIME
    mbedtls_mpi_shift_l(&C, 511);
    mbedtls_mpi_shift_r(&C, 511);

    B_T = TSF;
    mbedtls_mpi_shift_l(&C, 511);
    A_T = TSF;
    bignum_print("C << 511", &C);
    A("OP TIME : %d \n\n", A_T - B_T);

    B_T = TSF;
    mbedtls_mpi_shift_r(&C, 511);
    A_T = TSF;
    bignum_print("C >> 511", &C);
    A("OP TIME : %d \n\n", A_T - B_T);

    mbedtls_mpi_free( &A );
    mbedtls_mpi_free( &B );
    mbedtls_mpi_free( &C );
    mbedtls_mpi_free( &D );
    mbedtls_mpi_free( &E );
    mbedtls_mpi_free( &X );

    return( 0 );
}

SUBCMD(test,
	  bignum_op,
	  mbedtls_mpi_self_test_2,
	  "test bignum for ECC encrypt",
	  "test bignum");

#endif /* MBEDTLS_SELF_TEST */

#endif /* MBEDTLS_BIGNUM_C */
