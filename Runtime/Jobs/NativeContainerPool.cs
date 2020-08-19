/************************************************
* NativeContainerPool
* 
* Copyright (c) 2020 Yugo Fujioka
* 
* This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*************************************************/

using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace FUtility {
    /// <summary>
    /// Memory block in NativeArray
    /// </summary>
    internal unsafe struct NativeBlock {
        public int startIndex;
        public int size;
        public void* ptr;
    }

    /// <summary>
    /// Like memory pool for NativeArray
    /// </summary>
    [Il2CppSetOption(Option.NullChecks, false), Il2CppSetOption(Option.ArrayBoundsChecks, false), Il2CppSetOption(Option.DivideByZeroChecks, false)]
    public class NativeContainerPool<T> where T : struct {
        private NativeArray<T> array;
        private TaskSystem<NativeBlock> freePool, usedPool;

        private MatchHandler<NativeBlock> getFreeBlockHandler, getusedBlockHandler;
        private MatchHandler<NativeBlock> connectBlockTopHandler, connectBlockEndHandler;

        public NativeArray<T> nativeArray => this.array;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="arraySize">NativeArray size</param>
        /// <param name="blockCapacity">max block for pool</param>
        public NativeContainerPool(int arraySize, int blockCapacity) {
            if (arraySize < 0 || blockCapacity <= 0) {
                Debug.LogError("登録数が無効です");
                return;
            }
            this.array = new NativeArray<T>(arraySize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            this.freePool = new TaskSystem<NativeBlock>(blockCapacity);
            this.usedPool = new TaskSystem<NativeBlock>(blockCapacity);

            this.getFreeBlockHandler = new MatchHandler<NativeBlock>(this.GetFreeBlock);
            this.getusedBlockHandler = new MatchHandler<NativeBlock>(this.GetUsedBlock);
            this.connectBlockTopHandler = new MatchHandler<NativeBlock>(this.ConnectBlockTop);
            this.connectBlockEndHandler = new MatchHandler<NativeBlock>(this.ConnectBlockEnd);

            var block = new NativeBlock { startIndex = 0, size = arraySize };
            this.freePool.Attach(block);
        }

        /// <summary>
        /// Dispose native memories
        /// </summary>
        public void Dispose() {
            Debug.Assert(this.usedPool.count == 0, "Leak in NativeContainerPool");
            Debug.Assert(this.freePool.count == 1, "Unknown Error in NativeContainerPool");

            this.array.Dispose();
            this.freePool.Clear();
            this.usedPool.Clear();
        }

        /// <summary>
        /// Get native sub array as NestedNativeArray
        /// </summary>
        /// <param name="size">array size</param>
        /// <returns>success</returns>
        public unsafe bool AllocNestedArray(int size, out int index, out NestedNativeArray<T> nestedArray) {
            if (size == 0) {
                index = 0;
                nestedArray = default;
                return false;
            }

            NativeBlock block;
            this.needSize = size;
            if (this.freePool.Pickup(this.getFreeBlockHandler, out block)) {
                nestedArray = new NestedNativeArray<T>(this.array, block.startIndex, size);
                index = block.startIndex;

                var newBlock = new NativeBlock {
                    startIndex = block.startIndex,
                    size = size,
                    ptr = nestedArray.GetUnsafeReadOnlyPtr()
                };
                this.usedPool.Attach(newBlock);

                block.startIndex += size;
                block.size -= size;
                this.freePool.Attach(block);

                return true;
            }
            index = -1;
            nestedArray = default;
            return false;
        }
        /// <summary>
        /// Get native sub array
        /// </summary>
        /// <param name="size">array size</param>
        /// <returns>success</returns>
        public unsafe bool AllocSubArray(int size, out int index, out NativeArray<T> subArray) {
            if (size == 0) {
                index = 0;
                subArray = default;
                return false;
            }

            NativeBlock block;
            this.needSize = size;
            if (this.freePool.Pickup(this.getFreeBlockHandler, out block)) {
                subArray = this.array.GetSubArray(block.startIndex, size);
                index = block.startIndex;

                var newBlock = new NativeBlock {
                    startIndex = block.startIndex,
                    size = size,
                    ptr = NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(subArray)
                };
                this.usedPool.Attach(newBlock);

                block.startIndex += size;
                block.size -= size;
                this.freePool.Attach(block);

                return true;
            }
            index = -1;
            subArray = default;
            return false;
        }
        private int needSize = 0;
        private int GetFreeBlock(NativeBlock block) {
            if (block.size >= needSize)
                return 1;
            return 0;
        }

        /// <summary>
        /// Release allocated NestedNativeArray
        /// </summary>
        /// <param name="nestedArray">allocated array</param>
        public unsafe bool Free(NestedNativeArray<T> nestedArray) {
            if (nestedArray.Length == 0)
                return false;
            var ptr = nestedArray.GetUnsafeReadOnlyPtr();
            return this.Free(ptr);
        }

        /// <summary>
        /// Release allocated native sub array
        /// </summary>
        /// <param name="nestedArray">allocated array</param>
        public unsafe bool Free(NativeArray<T> array) {
            if (array.Length == 0)
                return false;
            var ptr = NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(array);
            return this.Free(ptr);
        }
        private unsafe bool Free(void* ptr) {
            this.freeAddress = ptr;

            // Find received pointer in used blocks.
            if (this.usedPool.Pickup(this.getusedBlockHandler, out this.connectBlock)) {
                NativeBlock block;
                // Add end of free blocks
                if (this.freePool.Pickup(this.connectBlockTopHandler, out block)) {
                    block.size += this.connectBlock.size;
                    this.connectBlock = block;
                }
                // Add top of free blocks
                if (this.freePool.Pickup(this.connectBlockEndHandler, out block)) {
                    this.connectBlock.size += block.size;
                }
                this.connectBlock.ptr = null; // safe delete
                this.freePool.Attach(this.connectBlock);

                return true;
            }
            return false;
        }
        private unsafe void* freeAddress = null;
        private unsafe int GetUsedBlock(NativeBlock block) {
            // NOTE: I suppose to use dozens blocks. So I don't recognize full scan as bottlenecks.
            if (this.freeAddress == block.ptr)
                return 1;
            return 0;
        }
        private NativeBlock connectBlock;
        private unsafe int ConnectBlockTop(NativeBlock block) {
            if (this.connectBlock.startIndex == (block.startIndex + block.size))

                return 1; 
            return 0;
        }
        private unsafe int ConnectBlockEnd(NativeBlock block) {
            if (block.startIndex == (this.connectBlock.startIndex + this.connectBlock.size))
                return 1;
            return 0;
        }
    }
}
