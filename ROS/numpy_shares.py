#!/usr/bin/env python3
import mmap
from typing import Tuple, List
import numpy as np
from posix_ipc import Semaphore, SharedMemory, O_CREAT
import struct
import time

class NumpyShareManager:
    def __init__(self, server=False):
        self.shares: List[SharedMemory] = []
        self.locks: List[Semaphore] = []
        self.mmaps: List[mmap.mmap] = []
        self.server = server
        
    def __enter__(self):
        return self
        
    def __exit__(self,  exc, exc2, exc3):
        self.stop()

    def create_new_share(self,name: str, size: int):
        share = SharedMemory(name, flags=O_CREAT, size=size)
        self.shares.append(share)
        map = mmap.mmap(share.fd, share.size)
        self.mmaps.append(map)
        return map

    def get_share(self, name: str):
        share = SharedMemory(name)
        self.shares.append(share)
        map = mmap.mmap(share.fd, share.size)
        self.mmaps.append(map)
        return map

    def get_lock(self, name: str, create=False):
        name = name+'-lock'
        if create:
            sem = Semaphore(name, flags = O_CREAT, initial_value=1)
        else:
            sem = Semaphore(name, initial_value=1)
        self.locks.append(sem)
        return sem

    def stop(self):
        print("stopping")
        for share in self.shares:
            share.close_fd()
        for lock in self.locks:
            lock.close()
        for map in self.mmaps:
            map.close()
        if self.server:
            for share in self.shares:
                share.unlink()
            for lock in self.locks:
                lock.unlink()
        time.sleep(0.1)

    @staticmethod
    def make_meta(data:np.ndarray) -> bytes:
        meta = struct.pack(f"Bc{data.ndim}I",data.ndim,bytes(data.dtype.char,"UTF8"),*data.shape)
        return meta
        
    @staticmethod
    def read_meta(arr: mmap.mmap) -> Tuple[int, np.dtype, tuple]:
        ndims = int(arr[0])
        type_char, *shape = struct.unpack(f"xc{ndims}I", arr)
        return np.dtype(type_char), shape
    
    def create_numpy_share(self, data: np.ndarray, name: str) -> np.ndarray:
        meta = self.make_meta(data)
        shm = self.create_new_share(name, size=data.nbytes)
        shm_meta = self.create_new_share(name+'-meta', size=len(meta))
        shm_meta[:len(meta)] = meta
        shm_arr = np.ndarray(data.shape, dtype=data.dtype, buffer=shm)
        shm_arr[:] = data[:]
        return shm_arr, self.get_lock(name, create=True)
        
    def get_numpy_share(self, name:str) -> np.ndarray:
        meta = self.get_share(name+'-meta')
        dt, shape = self.read_meta(meta)
        shm = self.get_share(name)
        arr = np.ndarray(shape, dtype=dt, buffer=shm)
        return arr, self.get_lock(name)
        
if __name__=="__main__":
    import time
    with NumpyShareManager(server=True) as man:
        here = np.array([[1,2,3],[3,4,5]])
        arr2, lock = man.create_numpy_share(here,"test")
        print(arr2)
        with lock:
            f = input("press enter")
            print(arr2)
        time.sleep(0.1)
        with lock:
            print(arr2)
