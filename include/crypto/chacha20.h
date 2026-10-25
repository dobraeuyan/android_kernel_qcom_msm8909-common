/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compatibility shim: this tree renamed the ChaCha20 API to the generic
 * ChaCha naming (crypto/chacha.h, mainline commit "crypto: chacha - unify").
 * linux-stable code (e.g. drivers/char/random.c) still includes
 * <crypto/chacha20.h> and uses the CHACHA20_* names, so map them here.
 */
#ifndef _CRYPTO_CHACHA20_H
#define _CRYPTO_CHACHA20_H

#include <crypto/chacha.h>

#define CHACHA20_IV_SIZE	CHACHA_IV_SIZE
#define CHACHA20_KEY_SIZE	CHACHA_KEY_SIZE
#define CHACHA20_BLOCK_SIZE	CHACHA_BLOCK_SIZE

/* chacha20_block() is already provided as an inline by <crypto/chacha.h>. */

/*
 * chacha_init_consts() was introduced alongside the linux-stable random.c
 * rewrite, after this tree's crypto/chacha.h snapshot; provide it here.
 * Initialises the first four state words with the "expand 32-byte k" magic.
 */
static inline void chacha_init_consts(u32 *state)
{
	state[0] = 0x61707865; /* "expa" */
	state[1] = 0x3320646e; /* "nd 3" */
	state[2] = 0x79622d32; /* "2-by" */
	state[3] = 0x6b206574; /* "te k" */
}

#endif /* _CRYPTO_CHACHA20_H */
