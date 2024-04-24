#include <iostream>
#include <chrono>
#include "common.hpp"
#include "utils.h"
#include "cuda_utils.h"
#include "logging.h"

#define USE_FP16 // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0 // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

static float data[1 * 3 * 640 * 640];
static float prob[1 * 6001];

static Logger gLogger;
struct Detection
{
    float bbox[4];
    float conf;
    float class_id;
};

struct yolo_data
{
    std::vector<Detection> detections;
};

char *my_classes[] = {"Chili"};

class YOLOV5
{
public:
    inline YOLOV5()
    {
        cudaSetDevice(DEVICE);
        engine_name = "../model/yolov5s.engine";
        std::ifstream file(engine_name, std::ios::binary);

        if (!file.good())
        {
            throw std::invalid_argument("read engine error ");
        }
        trtModelStream = nullptr;
        size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();

        runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        context = engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;
        assert(engine->getNbBindings() == 2);
        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
        outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(inputIndex == 0);
        assert(outputIndex == 1);
        // Create GPU buffers on device
        CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
        // Create stream
        CUDA_CHECK(cudaStreamCreate(&stream));
    }

    yolo_data yolov5(cv::Mat frame)
    {

        fcount++;
        // if (fcount < BATCH_SIZE && f + 1 != (int)file_names.size()) continue;
        for (int b = 0; b < fcount; b++)
        {
            // cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
            cv::Mat img = frame;
            if (img.empty())
                continue;
            cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row)
            {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col)
                {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }

        // Run inference
        auto start = std::chrono::system_clock::now(); // #��ȡģ��������ʼʱ��
        doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
        auto end = std::chrono::system_clock::now(); // #����ʱ��
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        int fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++)
        {
            auto &res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }
        auto &res = batch_res[0];
        // std::cout << res.size() << std::endl;
        // cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
        yolo_data result;

        for (size_t j = 0; j < res.size(); j++)
        {
            Detection obj;
            std::copy(std::begin(res[j].bbox), std::end(res[j].bbox), std::begin(obj.bbox));

            obj.class_id = res[j].class_id;
            obj.conf = res[j].conf;
            result.detections.push_back(obj);
        }

        fcount = 0;

        return result;
    }
    ~YOLOV5()
    {
        // Release stream and buffers
        cudaStreamDestroy(stream);
        CUDA_CHECK(cudaFree(buffers[inputIndex]));
        CUDA_CHECK(cudaFree(buffers[outputIndex]));
        // Destroy the engine
        context->destroy();
        engine->destroy();
        runtime->destroy();
    }

private:
    void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output, int batchSize)
    {
        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context.enqueue(batchSize, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
    }

    bool parse_args(int argc, char **argv, std::string &engine)
    {
        if (argc < 3)
            return false;
        if (std::string(argv[1]) == "-v" && argc == 3)
        {
            engine = std::string(argv[2]);
        }
        else
        {
            return false;
        }
        return true;
    }

    int key;
    int fcount = 0;

    // stuff we know about the network and the input/output blobs
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int CLASS_NUM = Yolo::CLASS_NUM;
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
    const char *INPUT_BLOB_NAME = "data";
    const char *OUTPUT_BLOB_NAME = "prob";
    std::string engine_name;
    size_t size;
    char *trtModelStream;

    void *buffers[2];

    int inputIndex;
    int outputIndex;

    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;
    cudaStream_t stream;
};