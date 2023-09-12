/*
 *  Copyright (C) 2017, Texas Instruments Incorporated, All Rights Reserved
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
 */
#include <stdlib.h>
#include <time.h>



#include <mbedtls/entropy.h>
#include <entropy_alt.h>
#include <ti/drivers/net/wifi/simplelink.h>


//int (*f_rng)(void *, unsigned char *, size_t)
int mbedtls_platform_entropy_poll(void *data, unsigned char *output, size_t len)
{
    size_t olen;
    int status;
    if (!output || !len) {
        return (MBEDTLS_ERR_ENTROPY_SOURCE_FAILED);
    }
    status = entropy_poll(data, output, len, &olen);
    //make a better solution for this if statment!!!!! and cheek the error
    if (status == 0 && olen == len)
    {
        return status;
    }
    return (status);
}

/* The entropy_poll function is a callback provided to the mbedtls_entropy_add_source().
 *
 * The arguments are:
 * - 'data' contains a pointer to the RNG_HandleTypeDef structure
 * - 'output' is a buffer of length 'len' allocated by the calling function
 * - 'olen' is areturn value indicating how many random bytes have ben placed in the 'output' buffer
 */


int entropy_poll(void *data, unsigned char *output, size_t len, size_t *olen)
{
    /*
     * Retrieve a buffer of true random numbers from the networking subsystem. Maximum buffer length is 172 bytes
     * for each retrieval. If the requested length exceeds 172 bytes, it is trimmed to 172 bytes
     * Status = sl_NetUtilGet(SL_NETUTIL_TRUE_RANDOM,0,buffer,&len);
     */
    int status;
    unsigned short length;
    length = (unsigned short)len;

    status = sl_NetUtilGet(SL_NETUTIL_TRUE_RANDOM,0,output,&(length));
    if (status==0)
    {
        *olen = len;
    }
    return status;



    /*!
        \brief     Function for getting configurations of utilities
        \param[in]     Option        Identifier of the specific "get" operation to perform
                                    - <b>SL_NETUTIL_CRYPTO_PUBLIC_KEY</b>  \n
                                    Used to retrieve the public key from an installed key-pair. \n
                                    Saved in a certain index.
                                    - <b>SL_NETUTIL_TRUE_RANDOM</b>  \n
                                    Generates a random number using the internal TRNG of the NWP. \n
        \param[in]     ObjID        ID of the relevant object that this set operation will be performed on
        \param[in,out] pValueLen    Pointer to the length of the value parameter\n
                                    On input - provides the length of the buffer that the application allocates, and
                                    will hold the output\n
                                    On output - provides the actual length of the received data
        \param[out]    pValues      Pointer to the buffer that the application allocates, and will hold
                                    the received data.
        \return        Zero on success, or negative error code on failure.
        \sa            sl_NetUtilSet sl_NetUtilCmd
        \note
        \warning
        \par    Examples
        - SL_NETUTIL_CRYPTO_PUBLIC_KEY:
        \code
        int16_t Status;
        uint8_t configOpt = 0;
        uint32_t objId = 0;
        uint16_t configLen = 0;
        uint8_t key_buf[256];

        configOpt = SL_NETUTIL_CRYPTO_PUBLIC_KEY;

        objId = 1;
        configLen = 255;
        //get the Public key
        Status = sl_NetUtilGet(configOpt, objId, key_buf, &configLen);
        \endcode

        - SL_NETUTIL_TRUE_RANDOM:
        \code
        uint32_t randNum;
        int32_t len = sizeof(uint32_t);

        sl_NetUtilGet(SL_NETUTIL_TRUE_RANDOM, 0, (uint8_t *)&randNum, &len);
        \endcode
        <br>
    */
}


psa_status_t mbedtls_psa_external_get_random( mbedtls_psa_external_random_context_t *context,
                                                uint8_t *output,
                                                size_t output_size, 
                                                size_t *output_length)
{
    int status;
    unsigned short length;
    length = (unsigned short)output_size;
    status = sl_NetUtilGet(SL_NETUTIL_TRUE_RANDOM,0,output,&(length));
    if (status==0)
    {
        *output_length = output_size;
    }
    return status;
}
//-------------------------------------------------------------------------