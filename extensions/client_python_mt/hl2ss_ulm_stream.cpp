
#define PY_SSIZE_T_CLEAN

#include <Python.h>
#include <hl2ss_ulm.h>

#define HL2SS_ULM_BEGIN  try
#define HL2SS_ULM_END(v) catch (std::exception const& e) { PyErr_SetString(PyExc_RuntimeError, e.what()); return v; } catch (...) { PyErr_SetString(PyExc_RuntimeError, "UNKNOWN"); return v; }

#define PyNone_New (Py_INCREF(Py_None), Py_None)

//-----------------------------------------------------------------------------
// create_configuration
//-----------------------------------------------------------------------------

template<typename T>
PyObject* create_configuration()
{
    throw std::runtime_error("Not implemented");
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_rm_vlc>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_vlc>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",   PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mode",    PyLong_FromUnsignedLong(c.mode));
    PyDict_SetItemString(r, "divisor", PyLong_FromUnsignedLong(c.divisor));
    PyDict_SetItemString(r, "profile", PyLong_FromUnsignedLong(c.profile));
    PyDict_SetItemString(r, "level",   PyLong_FromUnsignedLong(c.level));
    PyDict_SetItemString(r, "bitrate", PyLong_FromUnsignedLong(c.bitrate));
    PyDict_SetItemString(r, "options", PyNone_New);

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_rm_depth_ahat>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_ahat>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",      PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mode",       PyLong_FromUnsignedLong(c.mode));
    PyDict_SetItemString(r, "divisor",    PyLong_FromUnsignedLong(c.divisor));
    PyDict_SetItemString(r, "profile_z",  PyLong_FromUnsignedLong(c.profile_z));
    PyDict_SetItemString(r, "profile_ab", PyLong_FromUnsignedLong(c.profile_ab));
    PyDict_SetItemString(r, "level",      PyLong_FromUnsignedLong(c.level));
    PyDict_SetItemString(r, "bitrate",    PyLong_FromUnsignedLong(c.bitrate));
    PyDict_SetItemString(r, "options",    PyNone_New);

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_rm_depth_longthrow>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_longthrow>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",      PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mode",       PyLong_FromUnsignedLong(c.mode));
    PyDict_SetItemString(r, "divisor",    PyLong_FromUnsignedLong(c.divisor));
    PyDict_SetItemString(r, "png_filter", PyLong_FromUnsignedLong(c.png_filter));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_rm_imu>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_imu>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk", PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mode",  PyLong_FromUnsignedLong(c.mode));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_pv>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",          PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mode",           PyLong_FromUnsignedLong(c.mode));
    PyDict_SetItemString(r, "width",          PyLong_FromUnsignedLong(c.width));
    PyDict_SetItemString(r, "height",         PyLong_FromUnsignedLong(c.height));
    PyDict_SetItemString(r, "framerate",      PyLong_FromUnsignedLong(c.framerate));
    PyDict_SetItemString(r, "divisor",        PyLong_FromUnsignedLong(c.divisor));
    PyDict_SetItemString(r, "profile",        PyLong_FromUnsignedLong(c.profile));
    PyDict_SetItemString(r, "level",          PyLong_FromUnsignedLong(c.level));
    PyDict_SetItemString(r, "decoded_format", PyLong_FromUnsignedLong(c.decoded_format));
    PyDict_SetItemString(r, "bitrate",        PyLong_FromUnsignedLong(c.bitrate));
    PyDict_SetItemString(r, "options",        PyNone_New);

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_microphone>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_microphone>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",   PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "profile", PyLong_FromUnsignedLong(c.profile));
    PyDict_SetItemString(r, "level",   PyLong_FromUnsignedLong(c.level));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_si>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_si>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk", PyLong_FromUnsignedLongLong(c.chunk));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_eet>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_eet>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",     PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "framerate", PyLong_FromUnsignedLong(c.framerate));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_extended_audio>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_audio>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",           PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "mixer_mode",      PyLong_FromUnsignedLong(c.mixer_mode));
    PyDict_SetItemString(r, "loopback_gain",   PyFloat_FromDouble(c.loopback_gain));
    PyDict_SetItemString(r, "microphone_gain", PyFloat_FromDouble(c.microphone_gain));
    PyDict_SetItemString(r, "profile",         PyLong_FromUnsignedLong(c.profile));
    PyDict_SetItemString(r, "level",           PyLong_FromUnsignedLong(c.level));

    return r;
}

template<>
PyObject* create_configuration<hl2ss::ulm::configuration_extended_depth>()
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_depth>();

    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "chunk",       PyLong_FromUnsignedLongLong(c.chunk));
    PyDict_SetItemString(r, "media_index", PyLong_FromUnsignedLongLong(c.media_index));
    PyDict_SetItemString(r, "stride_mask", PyLong_FromUnsignedLongLong(c.stride_mask));
    PyDict_SetItemString(r, "mode",        PyLong_FromUnsignedLong(c.mode));
    PyDict_SetItemString(r, "divisor",     PyLong_FromUnsignedLong(c.divisor));
    PyDict_SetItemString(r, "profile_z",   PyLong_FromUnsignedLong(c.profile_z));

    return r;
}

static PyObject* create_configuration(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    uint16_t port;

    if (!PyArg_ParseTuple(args, "h", &port)) { return NULL; }

    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    return create_configuration<hl2ss::ulm::configuration_rm_vlc>();
    case hl2ss::stream_port::RM_DEPTH_AHAT:        return create_configuration<hl2ss::ulm::configuration_rm_depth_ahat>();
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   return create_configuration<hl2ss::ulm::configuration_rm_depth_longthrow>();
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  return create_configuration<hl2ss::ulm::configuration_rm_imu>();
    case hl2ss::stream_port::PERSONAL_VIDEO:
    case hl2ss::stream_port::EXTENDED_VIDEO:       return create_configuration<hl2ss::ulm::configuration_pv>();
    case hl2ss::stream_port::MICROPHONE:           return create_configuration<hl2ss::ulm::configuration_microphone>();
    case hl2ss::stream_port::SPATIAL_INPUT:        return create_configuration<hl2ss::ulm::configuration_si>();
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: return create_configuration<hl2ss::ulm::configuration_eet>();
    case hl2ss::stream_port::EXTENDED_AUDIO:       return create_configuration<hl2ss::ulm::configuration_extended_audio>();
    case hl2ss::stream_port::EXTENDED_DEPTH:       return create_configuration<hl2ss::ulm::configuration_extended_depth>();
    default:                                       throw std::runtime_error("Unsupported port");
    }
}
HL2SS_ULM_END(NULL)

static PyObject* dict_get_raw_item(PyObject* d, char const* key)
{
    PyObject *k = PyUnicode_FromString(key);
    PyObject *r = (PyDict_Contains(d, k) == 1) ? PyDict_GetItemWithError(d, k) : NULL;
    Py_DECREF(k);
    return r;
}

template<typename T, typename Ty>
Ty dict_get_item(PyObject* d, char const* key, T(*as)(PyObject*), Ty default_value)
{
    PyObject *i = dict_get_raw_item(d, key);
    if (!i) { return default_value; }
    T v = as(i);
    if ((v != (T)-1) || !PyErr_Occurred()) { return (Ty)v; }
    PyErr_Clear();
    return default_value;       
}

static bool decode_option_unit(PyObject* x, std::vector<uint64_t>& options)
{
    uint64_t o = PyLong_AsUnsignedLongLong(x);
    options.push_back(o);
    if ((o != (uint64_t)-1) || !PyErr_Occurred()) { return true; }
    PyErr_Clear();
    return false;
}

static bool decode_options(PyObject* d, char const* key, std::vector<uint64_t>& options)
{
    PyObject* o = dict_get_raw_item(d, key);

    if (!o || !PyDict_Check(o)) { return false; }

    PyObject* items = PyDict_Items(o);
    ssize_t count = PyList_Size(items);
    
    for (ssize_t i = 0; i < count; ++i)
    {
        PyObject* tuple = PyList_GetItem(items, i);

        if (!decode_option_unit(PyTuple_GetItem(tuple, 0), options)) { return false; }
        if (!decode_option_unit(PyTuple_GetItem(tuple, 1), options)) { return false; }
    }

    return true;
}

//-----------------------------------------------------------------------------
// open_stream
//-----------------------------------------------------------------------------

template<typename T>
std::unique_ptr<hl2ss::svc::source> open_stream(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    throw std::runtime_error("Not implemented");
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_rm_vlc>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_vlc>();
    std::vector<uint64_t> options;

    c.chunk   = dict_get_item(configuration, "chunk",   PyLong_AsUnsignedLongLong, c.chunk);
    c.mode    = dict_get_item(configuration, "mode",    PyLong_AsUnsignedLong,     c.mode);
    c.divisor = dict_get_item(configuration, "divisor", PyLong_AsUnsignedLong,     c.divisor);
    c.profile = dict_get_item(configuration, "profile", PyLong_AsUnsignedLong,     c.profile);
    c.level   = dict_get_item(configuration, "level",   PyLong_AsUnsignedLong,     c.level);
    c.bitrate = dict_get_item(configuration, "bitrate", PyLong_AsUnsignedLong,     c.bitrate);

    if (decode_options(configuration, "options", options))
    {
        c.options_size = options.size();
        c.options_data = options.data();
    }

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_rm_depth_ahat>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_ahat>();
    std::vector<uint64_t> options;

    c.chunk      = dict_get_item(configuration, "chunk",      PyLong_AsUnsignedLongLong, c.chunk);
    c.mode       = dict_get_item(configuration, "mode",       PyLong_AsUnsignedLong,     c.mode);
    c.divisor    = dict_get_item(configuration, "divisor",    PyLong_AsUnsignedLong,     c.divisor);
    c.profile_z  = dict_get_item(configuration, "profile_z",  PyLong_AsUnsignedLong,     c.profile_z);
    c.profile_ab = dict_get_item(configuration, "profile_ab", PyLong_AsUnsignedLong,     c.profile_ab);
    c.level      = dict_get_item(configuration, "level",      PyLong_AsUnsignedLong,     c.level);
    c.bitrate    = dict_get_item(configuration, "bitrate",    PyLong_AsUnsignedLong,     c.bitrate);

    if (decode_options(configuration, "options", options))
    {
        c.options_size = options.size();
        c.options_data = options.data();
    }

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_rm_depth_longthrow>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_depth_longthrow>();

    c.chunk      = dict_get_item(configuration, "chunk",      PyLong_AsUnsignedLongLong, c.chunk);
    c.mode       = dict_get_item(configuration, "mode",       PyLong_AsUnsignedLong,     c.mode);
    c.divisor    = dict_get_item(configuration, "divisor",    PyLong_AsUnsignedLong,     c.divisor);
    c.png_filter = dict_get_item(configuration, "png_filter", PyLong_AsUnsignedLong,     c.png_filter);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_rm_imu>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_rm_imu>();

    c.chunk = dict_get_item(configuration, "chunk", PyLong_AsUnsignedLongLong, c.chunk);
    c.mode  = dict_get_item(configuration, "mode",  PyLong_AsUnsignedLong,     c.mode);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_pv>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_pv>();
    std::vector<uint64_t> options;

    c.chunk          = dict_get_item(configuration, "chunk",          PyLong_AsUnsignedLongLong, c.chunk);
    c.mode           = dict_get_item(configuration, "mode",           PyLong_AsUnsignedLong,     c.mode);
    c.width          = dict_get_item(configuration, "width",          PyLong_AsUnsignedLong,     c.width);
    c.height         = dict_get_item(configuration, "height",         PyLong_AsUnsignedLong,     c.height);
    c.framerate      = dict_get_item(configuration, "framerate",      PyLong_AsUnsignedLong,     c.framerate);
    c.divisor        = dict_get_item(configuration, "divisor",        PyLong_AsUnsignedLong,     c.divisor);
    c.profile        = dict_get_item(configuration, "profile",        PyLong_AsUnsignedLong,     c.profile);
    c.level          = dict_get_item(configuration, "level",          PyLong_AsUnsignedLong,     c.level);
    c.decoded_format = dict_get_item(configuration, "decoded_format", PyLong_AsUnsignedLong,     c.decoded_format);
    c.bitrate        = dict_get_item(configuration, "bitrate",        PyLong_AsUnsignedLong,     c.bitrate);

    if (decode_options(configuration, "options", options))
    {
        c.options_size = options.size();
        c.options_data = options.data();
    }

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_microphone>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_microphone>();

    c.chunk   = dict_get_item(configuration, "chunk",   PyLong_AsUnsignedLongLong, c.chunk);
    c.profile = dict_get_item(configuration, "profile", PyLong_AsUnsignedLong,     c.profile);
    c.level   = dict_get_item(configuration, "level",   PyLong_AsUnsignedLong,     c.level);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_si>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_si>();

    c.chunk = dict_get_item(configuration, "chunk", PyLong_AsUnsignedLongLong, c.chunk);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_eet>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_eet>();

    c.chunk     = dict_get_item(configuration, "chunk",     PyLong_AsUnsignedLongLong, c.chunk);
    c.framerate = dict_get_item(configuration, "framerate", PyLong_AsUnsignedLong,     c.framerate);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_extended_audio>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_audio>();

    c.chunk           = dict_get_item(configuration, "chunk",           PyLong_AsUnsignedLongLong, c.chunk);
    c.mixer_mode      = dict_get_item(configuration, "mixer_mode",      PyLong_AsUnsignedLong,     c.mixer_mode);
    c.loopback_gain   = dict_get_item(configuration, "loopback_gain",   PyFloat_AsDouble,          c.loopback_gain);
    c.microphone_gain = dict_get_item(configuration, "microphone_gain", PyFloat_AsDouble,          c.microphone_gain);
    c.profile         = dict_get_item(configuration, "profile",         PyLong_AsUnsignedLong,     c.profile);
    c.level           = dict_get_item(configuration, "level",           PyLong_AsUnsignedLong,     c.level);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

template<>
std::unique_ptr<hl2ss::svc::source> open_stream<hl2ss::ulm::configuration_extended_depth>(char const* host, uint16_t port, uint64_t buffer_size, PyObject* configuration)
{
    auto c = hl2ss::svc::create_configuration<hl2ss::ulm::configuration_extended_depth>();

    c.chunk       = dict_get_item(configuration, "chunk",       PyLong_AsUnsignedLongLong, c.chunk);
    c.media_index = dict_get_item(configuration, "media_index", PyLong_AsUnsignedLongLong, c.media_index);
    c.stride_mask = dict_get_item(configuration, "stride_mask", PyLong_AsUnsignedLongLong, c.stride_mask);
    c.mode        = dict_get_item(configuration, "mode",        PyLong_AsUnsignedLong,     c.mode);
    c.divisor     = dict_get_item(configuration, "divisor",     PyLong_AsUnsignedLong,     c.divisor);
    c.profile_z   = dict_get_item(configuration, "profile_z",   PyLong_AsUnsignedLong,     c.profile_z);

    return hl2ss::svc::open_stream(host, port, buffer_size, &c);
}

static PyObject* open_stream(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    std::unique_ptr<hl2ss::svc::source> p;
    char const* host;
    uint16_t port;
    uint64_t buffer_size;
    PyObject* configuration;

    if (!PyArg_ParseTuple(args, "shLO!", &host, &port, &buffer_size, &PyDict_Type, &configuration)) { return NULL; }

    switch (port)
    {
    case hl2ss::stream_port::RM_VLC_LEFTFRONT:
    case hl2ss::stream_port::RM_VLC_LEFTLEFT:
    case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
    case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:    p = open_stream<hl2ss::ulm::configuration_rm_vlc>(host, port, buffer_size, configuration);             break;
    case hl2ss::stream_port::RM_DEPTH_AHAT:        p = open_stream<hl2ss::ulm::configuration_rm_depth_ahat>(host, port, buffer_size, configuration);      break;
    case hl2ss::stream_port::RM_DEPTH_LONGTHROW:   p = open_stream<hl2ss::ulm::configuration_rm_depth_longthrow>(host, port, buffer_size, configuration); break;
    case hl2ss::stream_port::RM_IMU_ACCELEROMETER:
    case hl2ss::stream_port::RM_IMU_GYROSCOPE:
    case hl2ss::stream_port::RM_IMU_MAGNETOMETER:  p = open_stream<hl2ss::ulm::configuration_rm_imu>(host, port, buffer_size, configuration);             break;
    case hl2ss::stream_port::PERSONAL_VIDEO:
    case hl2ss::stream_port::EXTENDED_VIDEO:       p = open_stream<hl2ss::ulm::configuration_pv>(host, port, buffer_size, configuration);                 break;
    case hl2ss::stream_port::MICROPHONE:           p = open_stream<hl2ss::ulm::configuration_microphone>(host, port, buffer_size, configuration);         break;
    case hl2ss::stream_port::SPATIAL_INPUT:        p = open_stream<hl2ss::ulm::configuration_si>(host, port, buffer_size, configuration);                 break;
    case hl2ss::stream_port::EXTENDED_EYE_TRACKER: p = open_stream<hl2ss::ulm::configuration_eet>(host, port, buffer_size, configuration);                break;
    case hl2ss::stream_port::EXTENDED_AUDIO:       p = open_stream<hl2ss::ulm::configuration_extended_audio>(host, port, buffer_size, configuration);     break;
    case hl2ss::stream_port::EXTENDED_DEPTH:       p = open_stream<hl2ss::ulm::configuration_extended_depth>(host, port, buffer_size, configuration);     break;
    default:                                       throw std::runtime_error("Unsupported port");
    }

    return PyLong_FromUnsignedLongLong((uint64_t)p.release());
}
HL2SS_ULM_END(NULL)

//-----------------------------------------------------------------------------
// get_by_
//-----------------------------------------------------------------------------

static PyObject* pack_frame(std::unique_ptr<hl2ss::svc::packet> packet)
{
    PyObject* r = PyDict_New();

    PyDict_SetItemString(r, "frame_stamp", PyLong_FromLongLong(packet->frame_stamp));
    PyDict_SetItemString(r, "status",      PyLong_FromLong(packet->status));
    PyDict_SetItemString(r, "timestamp",   PyLong_FromUnsignedLongLong(packet->timestamp));
    PyDict_SetItemString(r, "payload",     PyMemoryView_FromMemory((char*)packet->payload, packet->sz_payload, PyBUF_WRITE));
    PyDict_SetItemString(r, "pose",        PyMemoryView_FromMemory((char*)packet->pose, packet->pose ? sizeof(hl2ss::matrix_4x4) : 0, PyBUF_WRITE));
    PyDict_SetItemString(r, "_handle",     PyLong_FromUnsignedLongLong((uint64_t)packet.release()));

    return r;
}

static PyObject* get_by_index(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    uint64_t source;
    int64_t frame_stamp;

    if (!PyArg_ParseTuple(args, "LL", &source, &frame_stamp)) { return NULL; }
    return pack_frame(((hl2ss::svc::source*)source)->get_by_index(frame_stamp));
}
HL2SS_ULM_END(NULL)

static PyObject* get_by_timestamp(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    uint64_t source;
    uint64_t timestamp;
    int32_t time_preference;
    int32_t tiebreak_right;

    if (!PyArg_ParseTuple(args, "LLii", &source, &timestamp, &time_preference, &tiebreak_right)) { return NULL; }
    return pack_frame(((hl2ss::svc::source*)source)->get_by_timestamp(timestamp, time_preference, tiebreak_right != 0));
}
HL2SS_ULM_END(NULL)

static PyObject* get_pv_dimensions(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    uint64_t source;
    uint16_t width;
    uint16_t height;

    if (!PyArg_ParseTuple(args, "L", &source)) { return NULL; }

    ((hl2ss::svc::source*)source)->get_pv_dimensions(width, height);

    PyObject* r = PyTuple_New(2);

    PyTuple_SetItem(r, 0, PyLong_FromUnsignedLong(width));
    PyTuple_SetItem(r, 1, PyLong_FromUnsignedLong(height));

    return r;
}
HL2SS_ULM_END(NULL)

//-----------------------------------------------------------------------------
// release_
//-----------------------------------------------------------------------------

template<typename T>
PyObject* release(PyObject *args)
{
    uint64_t handle;
    if (!PyArg_ParseTuple(args, "L", &handle)) { return NULL; }
    delete (T*)handle;
    Py_RETURN_NONE;
}

static PyObject* release_packet(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    return release<hl2ss::svc::packet>(args);
}
HL2SS_ULM_END(NULL)

static PyObject* release_stream(PyObject *self, PyObject *args)
HL2SS_ULM_BEGIN
{
    return release<hl2ss::svc::source>(args);
}
HL2SS_ULM_END(NULL)

//-----------------------------------------------------------------------------
// Module
//-----------------------------------------------------------------------------

static PyMethodDef hl2ss_ulm_stream_methods[] =
{
    {"create_configuration", create_configuration, METH_VARARGS, "Create stream configuration"},
    {"open_stream",          open_stream,          METH_VARARGS, "Open HoloLens 2 stream"},
    {"get_by_index",         get_by_index,         METH_VARARGS, "Get packet by index"},
    {"get_by_timestamp",     get_by_timestamp,     METH_VARARGS, "Get packet by timestamp"},
    {"get_pv_dimensions",    get_pv_dimensions,    METH_VARARGS, "Get dynamic PV dimensions"},
    {"release_packet",       release_packet,       METH_VARARGS, "Release packet"},
    {"release_stream",       release_stream,       METH_VARARGS, "Close HoloLens 2 stream"},
    {NULL,                   NULL,                 0,            NULL},
};

static PyModuleDef hl2ss_ulm_stream_module =
{
    PyModuleDef_HEAD_INIT,
    "hl2ss_ulm_stream",
    "hl2ss ulm stream module",
    -1,
    hl2ss_ulm_stream_methods,
    NULL,
    NULL,
    NULL,
    NULL,
};

PyMODINIT_FUNC
PyInit_hl2ss_ulm_stream()
HL2SS_ULM_BEGIN
{
    hl2ss::svc::initialize();
    return PyModule_Create(&hl2ss_ulm_stream_module);
}
HL2SS_ULM_END(NULL)
