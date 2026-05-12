// Thin C wrapper around the OWL C++ API so Python ctypes can call it.
#include "owl.hpp"

static OWL::Context g_owl;

extern "C" {

// Returns 0 on success, -1 if open failed, -2 if initialize failed.
int owl_open(const char* server_ip) {
    if (g_owl.open(std::string(server_ip)) <= 0) return -1;
    if (g_owl.initialize("timebase=1,1000000") <= 0) {
        g_owl.close();
        return -2;
    }
    g_owl.streaming(1);
    return 0;
}

void owl_close() {
    if (g_owl.isOpen()) {
        g_owl.done();
        g_owl.close();
    }
}

// Non-blocking poll for one frame.
// Fills buf with groups of 9 floats per tracked rigid:
//   [id, pose[0..6], cond]
// where pose[0..6] = [x, y, z, qw, qx, qy, qz] in OWL native units (mm).
// Returns number of rigids written, 0 for no frame yet, -1 for OWL error.
int owl_poll(float* buf, int max_rigids) {
    const OWL::Event* event = g_owl.nextEvent(0);
    if (!event) return 0;
    if (event->type_id() == OWL::Type::ERROR) return -1;
    if (event->type_id() != OWL::Type::FRAME) return 0;

    OWL::Rigids rigids;
    if (event->find("rigids", rigids) == 0) return 0;

    int count = 0;
    for (const auto& r : rigids) {
        if (r.cond <= 0.0f) continue;
        if (count >= max_rigids) break;
        float* p = buf + count * 9;
        p[0] = static_cast<float>(r.id);
        p[1] = r.pose[0];
        p[2] = r.pose[1];
        p[3] = r.pose[2];
        p[4] = r.pose[3];
        p[5] = r.pose[4];
        p[6] = r.pose[5];
        p[7] = r.pose[6];
        p[8] = r.cond;
        count++;
    }
    return count;
}

} // extern "C"
