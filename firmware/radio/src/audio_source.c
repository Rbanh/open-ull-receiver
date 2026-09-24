#include "audio_source.h"
#include <string.h>
#include <limits.h>

_Static_assert(ATOMIC_INT_LOCK_FREE==2 && UINT_MAX==UINT32_MAX,
               "Audio publication needs lock-free 32-bit unsigned atomics");
_Static_assert((ULL_AUDIO_QUEUE_FRAMES & (ULL_AUDIO_QUEUE_FRAMES-1u))==0,
               "Power-of-two queue preserves indexing across counter wrap");

void ull_audio_source_init(struct ull_audio_source *s)
{
    if(!s)return;
    memset(s,0,sizeof(*s));
    atomic_init(&s->written,0);atomic_init(&s->read,0);atomic_init(&s->requested,false);
    atomic_init(&s->selected,0);atomic_init(&s->underflows,0);atomic_init(&s->discarded,0);
}

bool ull_audio_source_push(struct ull_audio_source *s,
                          const uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES])
{
    if(!s || !frames)return false;
    unsigned written=atomic_load_explicit(&s->written,memory_order_relaxed);
    unsigned read=atomic_load_explicit(&s->read,memory_order_acquire);
    if(written-read>=ULL_AUDIO_QUEUE_FRAMES)return false;
    memcpy(s->queue[written&(ULL_AUDIO_QUEUE_FRAMES-1u)],frames,
           sizeof(s->queue[0]));
    atomic_store_explicit(&s->written,written+1u,memory_order_release);
    return true;
}

int ull_audio_source_select(struct ull_audio_source *s,uint32_t event,
                           uint8_t frames[2][ULL_AIR_AUDIO_CHANNEL_BYTES])
{
    if(!s || !frames)return -1;
    atomic_store_explicit(&s->requested,true,memory_order_release);
    for(unsigned i=0;i<2;i++)if(s->latch[i].valid && s->latch[i].event==event){
        if(s->latch[i].present)memcpy(frames,s->latch[i].data,sizeof(s->latch[i].data));
        return s->latch[i].present?1:0;
    }
    if(s->selected_any && event<=s->last_event)return -1;
    unsigned read=atomic_load_explicit(&s->read,memory_order_relaxed);
    unsigned written=atomic_load_explicit(&s->written,memory_order_acquire);
    unsigned available=written-read;
    if(available>ULL_AUDIO_QUEUE_FRAMES)return -1;
    uint32_t skipped=s->selected_any?event-s->last_event-1u:0;
    unsigned discard=skipped<available?skipped:available;
    read+=discard;available-=discard;
    if(discard)atomic_fetch_add_explicit(&s->discarded,discard,memory_order_relaxed);
    /* Alternate by selection count, not event parity: a two-frame skip must
     * still preserve the immediately preceding owned frame's latch. */
    unsigned selected=atomic_load_explicit(&s->selected,memory_order_relaxed);
    unsigned slot=selected&1u;
    s->latch[slot].event=event;s->latch[slot].valid=true;
    s->latch[slot].present=available!=0;
    if(available){
        memcpy(s->latch[slot].data,s->queue[read&(ULL_AUDIO_QUEUE_FRAMES-1u)],
               sizeof(s->latch[slot].data));
        memcpy(frames,s->latch[slot].data,sizeof(s->latch[slot].data));
        read++;
    }else{
        memset(s->latch[slot].data,0,sizeof(s->latch[slot].data));
        atomic_fetch_add_explicit(&s->underflows,1,memory_order_relaxed);
    }
    atomic_store_explicit(&s->read,read,memory_order_release);
    s->last_event=event;s->selected_any=true;
    atomic_store_explicit(&s->selected,selected+1u,memory_order_relaxed);
    return available?1:0;
}

void ull_audio_source_stats(const struct ull_audio_source *s,
                            struct ull_audio_source_stats *out)
{
    if(!out)return;
    memset(out,0,sizeof(*out));
    if(!s)return;
    out->selected=atomic_load_explicit(&s->selected,memory_order_relaxed);
    out->underflows=atomic_load_explicit(&s->underflows,memory_order_relaxed);
    out->discarded=atomic_load_explicit(&s->discarded,memory_order_relaxed);
    /* Bounded, nonblocking read. A producer may append while sampled; that
     * count still existed between the two stable consumer-index reads. */
    for(unsigned attempt=0;attempt<3;attempt++){
        unsigned read=atomic_load_explicit(&s->read,memory_order_acquire);
        unsigned written=atomic_load_explicit(&s->written,memory_order_acquire);
        unsigned again=atomic_load_explicit(&s->read,memory_order_acquire);
        if(read==again && written-read<=ULL_AUDIO_QUEUE_FRAMES){
            out->queued=written-read;out->queue_valid=true;break;
        }
    }
}

bool ull_audio_source_rearm(struct ull_audio_source *s,
                           const struct ull_reconnect_barrier *barrier)
{
    if(!s || !barrier || !ull_reconnect_paused(barrier))return false;
    unsigned w=atomic_load(&s->written),r=atomic_load(&s->read);
    if(w-r>ULL_AUDIO_QUEUE_FRAMES)return false;
    atomic_fetch_add(&s->discarded,w-r);
    atomic_store(&s->read,w);
    atomic_store(&s->requested,false);
    memset(s->latch,0,sizeof(s->latch));
    s->last_event=0;s->selected_any=false;
    return true;
}
