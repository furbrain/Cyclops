#!/usr/bin/env python3

from typing import Tuple
import numpy as np
#from posix_ipc import Semaphore
import struct
from multiprocessing import shared_memory


class NumpyShareManager:
    def __init__(self, server=False):
        self.shares = []
        self.server = server
        
    def __enter__(self):
        return self
        
    def __exit__(self,  exc, exc2, exc3):
        self.stop()
            
    def stop(self):
        for f in self.shares:
            if self.server:
                f.unlink()
            else:
                f.close()
    
    @staticmethod
    def make_meta(data:np.ndarray) -> bytes:
        meta = struct.pack(f"Bc{data.ndim}I",data.ndim,bytes(data.dtype.char,"UTF8"),*data.shape)
        return meta
        
    @staticmethod
    def read_meta(arr: shared_memory.SharedMemory) -> Tuple[int, np.dtype, tuple]:
        buf = arr.buf
        ndims = int(buf[0])
        type_char, *shape = struct.unpack(f"xc{ndims}I", buf)
        return np.dtype(type_char), shape
    
    def create_numpy_share(self, data: np.ndarray, name: str) -> np.ndarray:
        meta = self.make_meta(data)
        print(meta)
        shm = shared_memory.SharedMemory(name, create=True, size=data.nbytes*2)
        shm_meta = shared_memory.SharedMemory(name+'-meta', create=True, size=len(meta))
        self.shares.extend([shm, shm_meta])
        print("shms created")
        shm_meta.buf[:len(meta)] = meta
        print("meta assigned")
        shm_arr = np.ndarray(data.shape, dtype=data.dtype, buffer=shm.buf)
        print("array created")
        shm_arr[:] = data[:]
        print("array written")
        print(shm_arr.nbytes, shm_arr.shape)
        print(shm.size)
        return shm_arr
        
    def get_numpy_share(self, name:str) -> np.ndarray:
        meta = shared_memory.SharedMemory(name+'-meta', create=False)
        dt, shape = self.read_meta(meta)
        shm = shared_memory.SharedMemory(name, create=False)
        #self.shares.extend([shm, meta])
        arr = np.ndarray(shape, dtype=dt, buffer=shm.buf)
        return arr
        
if __name__=="__main__":
    with NumpyShareManager(server=True) as man:
        here = np.array([[1,2,3],[3,4,5]])
        arr2 = man.create_numpy_share(here,"test")
        print(arr2)
