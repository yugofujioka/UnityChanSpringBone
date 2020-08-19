/************************************************
* TaskSystem
* 
* Copyright (c) 2017 Yugo Fujioka
* 
* This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*************************************************/

using System;
using UnityEngine;

namespace FUtility {
    #region IL2CPP OPTION
    // NOTE: Copy Attribute to avoid conflicts
    public enum Option {
        NullChecks = 1,
        ArrayBoundsChecks = 2,
        DivideByZeroChecks = 3,
    }

    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Method | AttributeTargets.Property, Inherited = false, AllowMultiple = true)]
    public class Il2CppSetOptionAttribute : Attribute {
        public Option Option { get; private set; }
        public object Value { get; private set; }

        public Il2CppSetOptionAttribute(Option option, object value) {
            Option = option;
            Value = value;
        }
    }
    #endregion

    public delegate bool OrderHandler<T>(T obj, int no); // true:ALIVE, false:DEAD
    public delegate int MatchHandler<T>(T obj); // 1:HIT, 0:MISS, -1:BREAK

    // タスク
    public sealed class Task<T> {
        public T item = default(T);
        public Task<T> prev = null;
        public Task<T> next = null;

        // 接続処理
        /// prev : 接続する前のノード
        /// next : 接続する後ろのノード
        public void Attach(Task<T> prev, Task<T> next) {
            this.prev = prev;
            this.next = next;
            if (prev != null)
                prev.next = this;
            if (next != null)
                next.prev = this;
        }

        /// 切断処理
        public void Detach() {
            if (this.prev != null)
                this.prev.next = this.next;
            if (this.next != null)
                this.next.prev = this.prev;

            this.prev = null;
            this.next = null;
        }
    }

    /// <summary>
    /// 双方向線形リストのタスクシステム
    /// NOTE: IL2CPPの例外判定を全無効にして高速化
    /// </summary>
    [Il2CppSetOption(Option.NullChecks, false), Il2CppSetOption(Option.ArrayBoundsChecks, false), Il2CppSetOption(Option.DivideByZeroChecks, false)]
    public sealed class TaskSystem<T> {
        private Task<T> top = null;    // 先端
        private Task<T> end = null;    // 終端
    
        private int capacity = 0;      // 最大タスク数
        private int freeCount = -1;    // 空きタスクインデックス
        private int actCount = 0;      // 稼動タスク数
        private Task<T>[] taskPool = null;    // 生成された全タスク
        private Task<T>[] activeTask = null;  // 待機中のプール

        /// <summary> 先端ノード </summary>
        public T first { get { return (this.top != null ? this.top.item : default(T)); } }
        /// <summary> 終端ノード </summary>
        public T last { get { return (this.end != null ? this.end.item : default(T)); } }
        // 稼動数
        public int count { get => this.actCount; }
    
    
        /// <summary>
        /// 双方向線形リストのタスクシステム
        /// </summary>
        /// <param name="capacity">最大タスク数</param>
        public TaskSystem(int capacity) {
            this.capacity = capacity;
            this.taskPool = new Task<T>[this.capacity];
            this.activeTask = new Task<T>[this.capacity];
            for (int i = 0; i < this.capacity; ++i) {
                this.taskPool[i] = new Task<T>();
                this.activeTask[i] = this.taskPool[i];
            }
            this.freeCount = this.capacity;
        }

        /// <summary>
        /// リストの全消去
        /// </summary>
        public void Clear() {
            this.freeCount = this.capacity;
            this.actCount = 0;
            this.top = null;
            this.end = null;
    
            for (int i = 0; i < this.capacity; ++i) {
                this.taskPool[i].prev = null;
                this.taskPool[i].next = null;
                this.activeTask[i] = this.taskPool[i];
            }
        }

        /// <summary>
        /// 接続
        /// </summary>
        /// <param name="item">追加するデータ</param>
        public bool Attach(T item) {
            Debug.Assert(item != null, "アタッチエラー");
            if (this.freeCount == 0) {
                Debug.LogWarning("Taskのキャパシティオーバーの為キャンセル");
                return false;
            }
    
            Task<T> task = this.activeTask[this.freeCount - 1];
            task.item = item;
    
            if (this.actCount > 0) {
                task.Attach(this.end, null);
                this.end = task;
            } else {
                task.Attach(null, null);
                this.end = task; this.top = task;
            }
    
            --this.freeCount;
            ++this.actCount;
            return true;
        }

        /// <summary>
        /// 接続解除
        /// </summary>
        /// <param name="task">切断するタスク</param>
        internal void Detach(Task<T> task) {
            if (task == this.top)
                this.top = task.next;
            if (task == this.end)
                this.end = task.prev;
            task.Detach();
    
            --this.actCount;
            ++this.freeCount;
            this.activeTask[this.freeCount-1] = task;
        }

        /// <summary>
        /// 接続解除
        /// </summary>
        /// <param name="match">条件式</param>
        public bool Detach(MatchHandler<T> match) {
            bool result = false;
            int no = 0;
            Task<T> now = null;
            for (Task<T> task = this.top; task != null && this.actCount > 0;) {
                // MEMO: 切断されても良い様に最初にノードを更新する
                now = task;
                task = task.next;

                int ret = match(now.item);
                if (ret != 0) {
                    this.Detach(now);
                    result = true;
                }
                // 中断
                if (ret < 0)
                    break;
                ++no;
            }
            return result;
        }

        /// <summary>
        /// 全タスクに指令
        /// </summary>
        /// <param name="order"></param>
        public void Order(OrderHandler<T> order) {
            int no = 0;
            Task<T> now = null;
            for (Task<T> task = this.top; task != null && this.actCount > 0;) {
                now = task;
                task = task.next;
                if (!order(now.item, no))
                    this.Detach(now);
                ++no;
            }
        }

        /// <summary>
        /// 条件指定に沿ったアイテムを返す
        /// </summary>
        /// <param name="command">指令</param>
        public bool Pickup(MatchHandler<T> match, out T result) {
            result = default(T);
            for (Task<T> task = this.top; task != null; task = task.next) {
                if (match(task.item) > 0) {
                    result = task.item;
                    this.Detach(task);
                    return true;
                }
            }

            return false;
        }
    }
}