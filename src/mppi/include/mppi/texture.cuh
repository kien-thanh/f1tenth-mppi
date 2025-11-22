#ifndef TEXTURE_CUH
#define TEXTURE_CUH

#include <stdexcept>
#include <iostream>

class Texture {
public:
    Texture(const float* data, float resolution, size_t width, size_t height, float origin_x, float origin_y) 
        : resolution_(resolution), width_(width), height_(height), origin_x_(origin_x), origin_y_(origin_y) 
    {
        if (!data || width <= 0 || height <= 0 || resolution <= 0.0f) {
            throw std::invalid_argument("Invalid parameters for Texture initialization.");
        }

        allocateTexture(data);
    }

    ~Texture() {
        cleanup();
    }

    __host__ __device__ cudaTextureObject_t getTextureObject() const { return texObj_; }
    __host__ __device__ float getResolution() const { return resolution_; }
    __host__ __device__ float getOriginX() const { return origin_x_; }
    __host__ __device__ float getOriginY() const { return origin_y_; }

private:
    void allocateTexture(const float* data) {
        // Allocate CUDA array in device memory
        cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float>();
        cudaMallocArray(&cuArray_, &channelDesc, width_, height_);

        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            throw std::runtime_error(std::string("Failed to allocate CUDA array: ") + cudaGetErrorString(err));
        }

        // Copy data located at address distMapHost in host memory to device memory
        // Pitch and width in bytes are the same because we have no padding
        cudaMemcpy2DToArray(cuArray_, 0, 0, data, width_  * sizeof(float), width_  * sizeof(float), height_, cudaMemcpyHostToDevice);

        err = cudaGetLastError();
        if (err != cudaSuccess) {
            cudaFreeArray(cuArray_);
            throw std::runtime_error(std::string("Failed to copy data to device: ") + cudaGetErrorString(err));
        }

        // Specify texture
        struct cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeArray;
        resDesc.res.array.array = cuArray_;

        // Specify texture object parameters
        struct cudaTextureDesc texDesc;
        memset(&texDesc, 0, sizeof(texDesc));
        texDesc.addressMode[0] = cudaAddressModeClamp;
        texDesc.addressMode[1] = cudaAddressModeClamp;
        texDesc.filterMode = cudaFilterModeLinear;
        texDesc.readMode = cudaReadModeElementType;
        texDesc.normalizedCoords = 0; // do not use normalized coordinates

        cudaCreateTextureObject(&texObj_, &resDesc, &texDesc, nullptr);

        err = cudaGetLastError();
        if (err != cudaSuccess) {
            cudaFreeArray(cuArray_);
            throw std::runtime_error(std::string("Failed to create texture object: ") + cudaGetErrorString(err));
        }
    }

    void cleanup() {
        if (texObj_) {
            cudaError_t err = cudaDestroyTextureObject(texObj_);
            if (err != cudaSuccess) {
                std::cerr << "Failed to destroy texture: " << cudaGetErrorString(err) << std::endl;
            }
            texObj_ = 0;
        }
        if (cuArray_) {
            cudaError_t err = cudaFreeArray(cuArray_);
            if (err != cudaSuccess) {
                std::cerr << "Failed to free array: " << cudaGetErrorString(err) << std::endl;
            }
            cuArray_ = nullptr;
        }
    }

    float resolution_;
    size_t width_;
    size_t height_;
    float origin_x_;
    float origin_y_;
    cudaArray_t cuArray_;
    cudaTextureObject_t texObj_;
};

__device__ __forceinline__ float sampleTexture(cudaTextureObject_t texObj, float x , float y, float resolution, float origin_x, float origin_y) {
    float offset = resolution / 2.0f;

    float u = (x - origin_x + offset) / resolution;
    float v = (y - origin_y + offset) / resolution;

    return tex2D<float>(texObj, u, v);
}

#endif // TEXTURE_CUH
