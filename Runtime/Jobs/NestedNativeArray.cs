using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace FUtility {
    /// <summary>
    /// NativeArray<NativeArray>する為のラッパー
    /// スレッドセーフかどうかのチェックを回避する為だけのもの
    /// </summary>
    /// <typeparam name="T">要素の型</typeparam>
    public unsafe struct NestedNativeArray<T> where T : struct {
        private void* ptr;
        private int length;

        /// <summary>
        /// NativeArrayからのラップ
        /// </summary>
        /// <param name="array">元の配列</param>
        /// <param name="startIndex">開始インデックス</param>
        /// <param name="length">サイズ</param>
        public NestedNativeArray(NativeArray<T> array, int startIndex, int length) {
            if (array.Length == 0 || length == 0) {
                this.ptr = null;
                this.length = 0;
                return;
            }
            var subArray = array.GetSubArray(startIndex, length);
            this.ptr = NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(subArray);
            this.length = subArray.Length;
        }

        /// <summary>
        /// 全体長
        /// </summary>
        public int Length => this.length;

        /// <summary>
        /// NativeArrayのように使える
        /// </summary>
        /// <param name="index">要素インデックス</param>
        /// <returns>要素</returns>
        public T this[int index] {
            get {
#if UNITY_EDITOR
                if (index < 0 || this.length <= index) {
                    UnityEngine.Debug.LogError($"Index {index} is out of range (must be between 0 and {this.length - 1}).");
                    return default;
                }
#endif
                return UnsafeUtility.ReadArrayElement<T>(this.ptr, index);
            }

            [WriteAccessRequired]
            set {
#if UNITY_EDITOR
                if (index < 0 || this.length <= index) {
                    UnityEngine.Debug.LogError($"Index {index} is out of range (must be between 0 and {this.length - 1}).");
                    return;
                }
#endif
                UnsafeUtility.WriteArrayElement(this.ptr, index, value);
            }
        }

        /// <summary>
        /// 配列のポインタ
        /// </summary>
        public unsafe void* GetUnsafeReadOnlyPtr() {
            return this.ptr;
        }
    }
}
