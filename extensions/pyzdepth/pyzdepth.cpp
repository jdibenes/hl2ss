
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <zdepth.hpp>

typedef 
struct
{
    PyObject_HEAD
    zdepth::DepthCompressor zddc;
} 
DepthCompressor;

PyObject *DepthCompressor_new(PyTypeObject *type, PyObject *, PyObject *)
{
    DepthCompressor *self = (DepthCompressor*) type->tp_alloc(type, 0);
    return (PyObject*) self;
}

int DepthCompressor_init(PyObject *self, PyObject *args, PyObject *kwds)
{
    return 0;
}

void DepthCompressor_dealloc(DepthCompressor *self)
{
    Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject *DepthCompressor_Compress(PyObject *self, PyObject *args)
{
    int width;
    int height;
    char const *unquantized_depth;
    Py_ssize_t size;
    int keyframe;

    if (!PyArg_ParseTuple(args, "iiy#p", &width, &height, &unquantized_depth, &size, &keyframe)) {return NULL;}

    DepthCompressor* _self = reinterpret_cast<DepthCompressor*>(self);
    
    std::vector<uint8_t> compressed;

    zdepth::DepthResult result = _self->zddc.Compress(width, height, (uint16_t*)unquantized_depth, compressed, keyframe != 0);

    return Py_BuildValue("iy#", (int)result, compressed.data(), compressed.size() * sizeof(uint8_t));
}

PyObject *DepthCompressor_Decompress(PyObject *self, PyObject *args)
{
    char const *data;
    Py_ssize_t size;

    if (!PyArg_ParseTuple(args, "y#", &data, &size)) {return NULL;}

    DepthCompressor * _self = reinterpret_cast<DepthCompressor*>(self);

    std::vector<uint8_t> compressed(data, data + size);
    int width;
    int height;
    std::vector<uint16_t> depth_out;
    
    zdepth::DepthResult result = _self->zddc.Decompress(compressed, width, height, depth_out);

    return Py_BuildValue("iiiy#", (int)result, width, height, depth_out.data(), depth_out.size() * sizeof(uint16_t));
}

static PyMethodDef DepthCompressor_methods[] =
{
    {"Compress",   (PyCFunction)DepthCompressor_Compress,   METH_VARARGS, PyDoc_STR("Compress uint16 depth frame")},
    {"Decompress", (PyCFunction)DepthCompressor_Decompress, METH_VARARGS, PyDoc_STR("Decompress uint16 depth frame")},
    {NULL, NULL, 0, NULL}
};

static PyType_Slot DepthCompressor_slots[] = 
{
    {Py_tp_new, (void*)DepthCompressor_new},
    {Py_tp_init, (void*)DepthCompressor_init},
    {Py_tp_dealloc, (void*)DepthCompressor_dealloc},
    {Py_tp_methods, DepthCompressor_methods},
    {0, 0}
};

static PyType_Spec DepthCompressor_spec =
{
    "DepthCompressor",
    sizeof(DepthCompressor),
    0,
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    DepthCompressor_slots
};

PyModuleDef DepthCompressor_module =
{
    PyModuleDef_HEAD_INIT,
    "pyzdepth",
    "Wrapper for Zdepth",
    -1,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

PyMODINIT_FUNC PyInit_pyzdepth()
{
    PyObject* module = PyModule_Create(&DepthCompressor_module);
    PyObject* depthcompressor = PyType_FromSpec(&DepthCompressor_spec);
    if (depthcompressor == NULL) {return NULL;}
    Py_INCREF(depthcompressor);
    if (PyModule_AddObject(module, "DepthCompressor", depthcompressor) < 0)
    {
        Py_DECREF(depthcompressor);
        Py_DECREF(module);
        return NULL;
    }
    return module;
}
