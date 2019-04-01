// THIS IS AN AUTOMATICALLY GENERATED FILE.
// DO NOT MODIFY BY HAND!!
//
// Generated by zcm-gen

#include <string.h>
#ifndef ZCM_EMBEDDED
#include <stdio.h>
#endif
#include "sensor_accel_t.h"

static int __sensor_accel_t_hash_computed = 0;
static uint64_t __sensor_accel_t_hash;

uint64_t __sensor_accel_t_hash_recursive(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __sensor_accel_t_get_hash)
            return 0;

    __zcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__sensor_accel_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x2d589603ac8e0da0LL
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __sensor_accel_t_get_hash(void)
{
    if (!__sensor_accel_t_hash_computed) {
        __sensor_accel_t_hash = (int64_t)__sensor_accel_t_hash_recursive(NULL);
        __sensor_accel_t_hash_computed = 1;
    }

    return __sensor_accel_t_hash;
}

int __sensor_accel_t_encode_array(void* buf, uint32_t offset, uint32_t maxlen, const sensor_accel_t* p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].a_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].a_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].a_z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].t), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int sensor_accel_t_encode(void* buf, uint32_t offset, uint32_t maxlen, const sensor_accel_t* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_accel_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __sensor_accel_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t __sensor_accel_t_encoded_array_size(const sensor_accel_t* p, uint32_t elements)
{
    uint32_t size = 0, element;
    for (element = 0; element < elements; ++element) {

        size += __double_encoded_array_size(&(p[element].a_x), 1);

        size += __double_encoded_array_size(&(p[element].a_y), 1);

        size += __double_encoded_array_size(&(p[element].a_z), 1);

        size += __double_encoded_array_size(&(p[element].t), 1);

    }
    return size;
}

uint32_t sensor_accel_t_encoded_size(const sensor_accel_t* p)
{
    return 8 + __sensor_accel_t_encoded_array_size(p, 1);
}

int __sensor_accel_t_decode_array(const void* buf, uint32_t offset, uint32_t maxlen, sensor_accel_t* p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].a_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].a_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].a_z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].t), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __sensor_accel_t_decode_array_cleanup(sensor_accel_t* p, uint32_t elements)
{
    uint32_t element;
    for (element = 0; element < elements; ++element) {

        __double_decode_array_cleanup(&(p[element].a_x), 1);

        __double_decode_array_cleanup(&(p[element].a_y), 1);

        __double_decode_array_cleanup(&(p[element].a_z), 1);

        __double_decode_array_cleanup(&(p[element].t), 1);

    }
    return 0;
}

int sensor_accel_t_decode(const void* buf, uint32_t offset, uint32_t maxlen, sensor_accel_t* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_accel_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __sensor_accel_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sensor_accel_t_decode_cleanup(sensor_accel_t* p)
{
    return __sensor_accel_t_decode_array_cleanup(p, 1);
}

uint32_t __sensor_accel_t_clone_array(const sensor_accel_t* p, sensor_accel_t* q, uint32_t elements)
{
    uint32_t n = 0, element;
    for (element = 0; element < elements; ++element) {

        n += __double_clone_array(&(p[element].a_x), &(q[element].a_x), 1);

        n += __double_clone_array(&(p[element].a_y), &(q[element].a_y), 1);

        n += __double_clone_array(&(p[element].a_z), &(q[element].a_z), 1);

        n += __double_clone_array(&(p[element].t), &(q[element].t), 1);

    }
    return n;
}

sensor_accel_t* sensor_accel_t_copy(const sensor_accel_t* p)
{
    sensor_accel_t* q = (sensor_accel_t*) malloc(sizeof(sensor_accel_t));
    __sensor_accel_t_clone_array(p, q, 1);
    return q;
}

void sensor_accel_t_destroy(sensor_accel_t* p)
{
    __sensor_accel_t_decode_array_cleanup(p, 1);
    free(p);
}

int sensor_accel_t_publish(zcm_t* zcm, const char* channel, const sensor_accel_t* p)
{
      uint32_t max_data_size = sensor_accel_t_encoded_size (p);
      uint8_t* buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = sensor_accel_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = zcm_publish (zcm, channel, buf, (uint32_t)data_size);
      free (buf);
      return status;
}

struct _sensor_accel_t_subscription_t {
    sensor_accel_t_handler_t user_handler;
    void* userdata;
    zcm_sub_t* z_sub;
};
static
void sensor_accel_t_handler_stub (const zcm_recv_buf_t* rbuf,
                            const char* channel, void* userdata)
{
    int status;
    sensor_accel_t p;
    memset(&p, 0, sizeof(sensor_accel_t));
    status = sensor_accel_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr, "error %d decoding sensor_accel_t!!!\n", status);
        #endif
        return;
    }

    sensor_accel_t_subscription_t* h = (sensor_accel_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    sensor_accel_t_decode_cleanup (&p);
}

sensor_accel_t_subscription_t* sensor_accel_t_subscribe (zcm_t* zcm,
                    const char* channel,
                    sensor_accel_t_handler_t f, void* userdata)
{
    sensor_accel_t_subscription_t* n = (sensor_accel_t_subscription_t*)
                       malloc(sizeof(sensor_accel_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->z_sub = zcm_subscribe (zcm, channel,
                              sensor_accel_t_handler_stub, n);
    if (n->z_sub == NULL) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr,"couldn't reg sensor_accel_t ZCM handler!\n");
        #endif
        free (n);
        return NULL;
    }
    return n;
}

int sensor_accel_t_unsubscribe(zcm_t* zcm, sensor_accel_t_subscription_t* hid)
{
    int status = zcm_unsubscribe (zcm, hid->z_sub);
    if (0 != status) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr,
           "couldn't unsubscribe sensor_accel_t_handler %p!\n", hid);
        #endif
        return -1;
    }
    free (hid);
    return 0;
}
