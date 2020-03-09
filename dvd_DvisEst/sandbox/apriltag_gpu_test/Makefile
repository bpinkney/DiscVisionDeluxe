BIN               := mv 
CXXFLAGS          := -O3 -g

CUDA_INSTALL_PATH ?= /usr/local/cuda

NVCC ?= $(CUDA_INSTALL_PATH)/bin/nvcc
INCD = -I"$(CUDA_INSTALL_PATH)/include" -I"./inc"
LIBS = -lcuda -L"$(CUDA_INSTALL_PATH)/lib64" -lcudart -lcublas -lcufft -L"$(CUDA_INSTALL_PATH)/samples/common/inc -lstdc++ -lpthread
CUDA_SDK?=3
COMMONFLAGS = -DCUDA_SDK=$(CUDA_SDK)
NVCCFLAGS := --ptxas-options=-v -O3 -G -g -std=c++11
VPATH=src

export PATH=$PATH:/usr/local/cuda/bin:$$(PATH)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:$$(LD_LIBRARY_PATH)

#apriltag stuff
ISSAC_APRILTAG_INC = -I"../../lib/apriltag_gpu/nvapriltags"
ISSAC_APRILTAG_LIB = -L"../../lib/apriltag_gpu/lib_x64 -lapril_tagging"
INCD += $(ISSAC_APRILTAG_INC)
LIBS += $(ISSAC_APRILTAG_LIB)

# files
CPP_SOURCES       := 
CU_SOURCES        := apriltag_gpu_test.cu
HEADERS           := $(wildcard *.h)
CPP_OBJS          := $(patsubst %.cpp, %.o, $(CPP_SOURCES))
CU_OBJS           := $(patsubst %.cu, %.cu_o, $(CU_SOURCES))

%.cu_o : %.cu
	$(NVCC) $(NVCCFLAGS) -c $(INCD) -o $@ $<

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCD) -o $@ $<

$(BIN): $(CPP_OBJS) $(CU_OBJS)
	$(CXX) -o $(BIN) $(CU_OBJS) $(CPP_OBJS) $(LDFLAGS) $(INCD) $(LIBS)

clean:
	rm -f $(BIN) *.o *.cu_o