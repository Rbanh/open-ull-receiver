#include "air_frame.h"
#include <string.h>

static uint32_t crc32_continue(uint32_t seed, const uint8_t *data, size_t size)
{
    uint32_t crc = ~seed;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? UINT32_C(0xedb88320) : 0u);
    }
    return ~crc;
}

static bool sizes_valid(const uint8_t *sizes, size_t count)
{
    if (!sizes || count == 0 || count > AIR_MAX_STREAMS)
        return false;
    for (size_t i = 0; i < count; ++i) {
        if (sizes[i] == 0 || sizes[i] > AIR_MAX_RECORD_PAYLOAD)
            return false;
        for (size_t j = 0; j < i; ++j)
            if (sizes[j] == sizes[i])
                return false;
    }
    return true;
}

int air_downlink_build(uint8_t *dst, size_t capacity,
                       const struct air_tx_record *records, size_t count,
                       const uint8_t *sizes, size_t size_count, uint8_t flags,
                       size_t *written, uint8_t *stream_mask)
{
    if (!dst || !records || !written || !stream_mask || count == 0 ||
        count > AIR_MAX_STREAMS || (flags & ~0x70u) || !sizes_valid(sizes, size_count))
        return AIR_FRAME_ARGUMENT;
    size_t total = 1;
    uint8_t mask = 0, selector = 0;
    bool selected = false;
    for (size_t i = 0; i < count; ++i) {
        const struct air_tx_record *r = &records[i];
        if (r->stream_id >= AIR_MAX_STREAMS || r->tx_sequence > 15 ||
            r->ack_sequence > 15 || r->kind > 3 ||
            (i && r->stream_id <= records[i - 1].stream_id) ||
            r->length > AIR_MAX_RECORD_PAYLOAD || (r->length && !r->payload))
            return AIR_FRAME_ARGUMENT;
        if (r->length) {
            size_t index = 0;
            while (index < size_count && sizes[index] != r->length)
                ++index;
            if (index == size_count || (selected && index != selector))
                return AIR_FRAME_ARGUMENT;
            selector = (uint8_t)index;
            selected = true;
        }
        mask |= (uint8_t)(1u << r->stream_id);
        total += 4 + r->length;
    }
    if (total > AIR_MAX_AGGREGATE || total > capacity)
        return AIR_FRAME_CAPACITY;
    dst[0] = flags | selector;
    size_t cursor = 1;
    for (size_t i = 0; i < count; ++i) {
        const struct air_tx_record *r = &records[i];
        uint8_t header[2] = {
            (uint8_t)((r->tx_sequence << 4) | r->ack_sequence),
            (uint8_t)(r->kind | (r->length ? 0u : 4u)),
        };
        uint32_t crc = crc32_continue(UINT32_C(0xffffff00) | r->stream_id,
                                      r->payload, r->length);
        crc = crc32_continue(crc, header, sizeof(header));
        memcpy(dst + cursor, header, sizeof(header));
        cursor += sizeof(header);
        if (r->length)
            memcpy(dst + cursor, r->payload, r->length);
        cursor += r->length;
        dst[cursor++] = (uint8_t)crc;
        dst[cursor++] = (uint8_t)(crc >> 8);
    }
    *written = cursor;
    *stream_mask = mask;
    return AIR_FRAME_OK;
}

static int uplink_parse(const uint8_t *src, size_t length, uint8_t stream_mask,
                     const uint8_t *sizes, size_t size_count,
                     struct air_rx_record *records, size_t capacity,
                     size_t *record_count,bool has_control,struct air_control_record *control)
{
    if (!src || !records || !record_count || !stream_mask || (stream_mask & 0xf0u) ||
        !sizes_valid(sizes, size_count))
        return AIR_FRAME_ARGUMENT;
    if (!length || length > AIR_MAX_AGGREGATE)
        return AIR_FRAME_MALFORMED;
    size_t selector = src[0] & 3u;
    if (selector >= size_count)
        return AIR_FRAME_MALFORMED;
    struct air_rx_record decoded[AIR_MAX_STREAMS];
    size_t cursor = 1, count = 0;
    for (uint8_t id = 0; id < AIR_MAX_STREAMS; ++id) {
        if (!(stream_mask & (1u << id)))
            continue;
        if (length - cursor < 2)
            return AIR_FRAME_MALFORMED;
        const uint8_t *header = src + cursor;
        cursor += 2;
        size_t bytes = (header[1] & 0x0cu) ? 0 : sizes[selector];
        if (bytes > length - cursor)
            return AIR_FRAME_MALFORMED;
        decoded[count++] = (struct air_rx_record) {
            .payload = src + cursor, .length = bytes, .stream_id = id,
            .tx_sequence = header[0] >> 4, .ack_sequence = header[0] & 15u,
            .kind = header[1] & 3u, .flags = header[1],
        };
        cursor += bytes;
    }
    struct air_control_record tail={0};
    if(has_control){
        if(!control || length-cursor<2 || length-cursor-2!=src[cursor+1] ||
           !(src[cursor]&3u) || (!src[cursor+1] && (src[cursor]&3u)!=1))
            return AIR_FRAME_MALFORMED;
        tail=(struct air_control_record){.payload=src+cursor+2,
            .length=src[cursor+1],.header=src[cursor],.present=true};
        cursor=length;
    }
    if (cursor != length)
        return AIR_FRAME_MALFORMED;
    if (count > capacity)
        return AIR_FRAME_CAPACITY;
    memcpy(records, decoded, count * sizeof(decoded[0]));
    *record_count = count;
    if(control)*control=tail;
    return AIR_FRAME_OK;
}

int air_uplink_parse(const uint8_t *src, size_t length, uint8_t stream_mask,
                     const uint8_t *sizes, size_t size_count,
                     struct air_rx_record *records, size_t capacity,size_t *count)
{ return uplink_parse(src,length,stream_mask,sizes,size_count,records,capacity,count,false,NULL); }

int air_uplink_parse_control(const uint8_t *src,size_t length,uint8_t header,
                             const uint8_t *sizes,size_t size_count,
                             struct air_rx_record *records,size_t capacity,size_t *count,
                             struct air_control_record *control)
{
    if(!control)return AIR_FRAME_ARGUMENT;
    return uplink_parse(src,length,(uint8_t)((header>>3)&15),sizes,size_count,
                        records,capacity,count,(header&1u)!=0,control);
}
