// THIS IS AN AUTOMATICALLY GENERATED FILE.
// DO NOT MODIFY BY HAND!!
//
// Generated by zcm-gen

#include <string.h>
#ifndef ZCM_EMBEDDED
#include <stdio.h>
#endif
#include "msg_t.h"

static int __msg_t_hash_computed = 0;
static uint64_t __msg_t_hash;

uint64_t __msg_t_hash_recursive(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __msg_t_get_hash)
            return 0;

    __zcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__msg_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x4c9c80b2afc26e46LL
         + __string_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __msg_t_get_hash(void)
{
    if (!__msg_t_hash_computed) {
        __msg_t_hash = (int64_t)__msg_t_hash_recursive(NULL);
        __msg_t_hash_computed = 1;
    }

    return __msg_t_hash;
}

int __msg_t_encode_array(void* buf, uint32_t offset, uint32_t maxlen, const msg_t* p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &(p[element].str), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int msg_t_encode(void* buf, uint32_t offset, uint32_t maxlen, const msg_t* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __msg_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __msg_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t __msg_t_encoded_array_size(const msg_t* p, uint32_t elements)
{
    uint32_t size = 0, element;
    for (element = 0; element < elements; ++element) {

        size += __string_encoded_array_size(&(p[element].str), 1);

    }
    return size;
}

uint32_t msg_t_encoded_size(const msg_t* p)
{
    return 8 + __msg_t_encoded_array_size(p, 1);
}

int __msg_t_decode_array(const void* buf, uint32_t offset, uint32_t maxlen, msg_t* p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        thislen = __string_decode_array(buf, offset + pos, maxlen - pos, &(p[element].str), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __msg_t_decode_array_cleanup(msg_t* p, uint32_t elements)
{
    uint32_t element;
    for (element = 0; element < elements; ++element) {

        __string_decode_array_cleanup(&(p[element].str), 1);

    }
    return 0;
}

int msg_t_decode(const void* buf, uint32_t offset, uint32_t maxlen, msg_t* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __msg_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __msg_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int msg_t_decode_cleanup(msg_t* p)
{
    return __msg_t_decode_array_cleanup(p, 1);
}

uint32_t __msg_t_clone_array(const msg_t* p, msg_t* q, uint32_t elements)
{
    uint32_t n = 0, element;
    for (element = 0; element < elements; ++element) {

        n += __string_clone_array(&(p[element].str), &(q[element].str), 1);

    }
    return n;
}

msg_t* msg_t_copy(const msg_t* p)
{
    msg_t* q = (msg_t*) malloc(sizeof(msg_t));
    __msg_t_clone_array(p, q, 1);
    return q;
}

void msg_t_destroy(msg_t* p)
{
    __msg_t_decode_array_cleanup(p, 1);
    free(p);
}

int msg_t_publish(zcm_t* zcm, const char* channel, const msg_t* p)
{
      uint32_t max_data_size = msg_t_encoded_size (p);
      uint8_t* buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = msg_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = zcm_publish (zcm, channel, buf, (uint32_t)data_size);
      free (buf);
      return status;
}

struct _msg_t_subscription_t {
    msg_t_handler_t user_handler;
    void* userdata;
    zcm_sub_t* z_sub;
};
static
void msg_t_handler_stub (const zcm_recv_buf_t* rbuf,
                            const char* channel, void* userdata)
{
    int status;
    msg_t p;
    memset(&p, 0, sizeof(msg_t));
    status = msg_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr, "error %d decoding msg_t!!!\n", status);
        #endif
        return;
    }

    msg_t_subscription_t* h = (msg_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    msg_t_decode_cleanup (&p);
}

msg_t_subscription_t* msg_t_subscribe (zcm_t* zcm,
                    const char* channel,
                    msg_t_handler_t f, void* userdata)
{
    msg_t_subscription_t* n = (msg_t_subscription_t*)
                       malloc(sizeof(msg_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->z_sub = zcm_subscribe (zcm, channel,
                              msg_t_handler_stub, n);
    if (n->z_sub == NULL) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr,"couldn't reg msg_t ZCM handler!\n");
        #endif
        free (n);
        return NULL;
    }
    return n;
}

int msg_t_unsubscribe(zcm_t* zcm, msg_t_subscription_t* hid)
{
    int status = zcm_unsubscribe (zcm, hid->z_sub);
    if (0 != status) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr,
           "couldn't unsubscribe msg_t_handler %p!\n", hid);
        #endif
        return -1;
    }
    free (hid);
    return 0;
}

