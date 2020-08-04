using System.Collections.Generic;
using Unity.Jobs;
using UnityEngine;
using Unity.Collections;


namespace Unity.Animations.SpringBones.JobExtend {
	/// <summary>
	/// SpringBoneのJob発行
	/// NOTE: 頻繁にSchedulerに登録と解除を行う場合はListは避けるべき
	/// </summary>
	public class SpringJobScheduler : MonoBehaviour {
		private static SpringJobScheduler instance = null;

		private List<SpringIJobManager> managers = new List<SpringIJobManager>();
		private NativeArray<JobHandle> handles;
		private bool scheduled = false;

		void OnDestroy() {
			if (instance == null)
				return;

			this.managers.Clear();
			
			if (this.scheduled)
				this.handles.Dispose();
			instance = null;
		}

		void LateUpdate() {
			if (!this.scheduled)
				return;

			if (this.handles.Length > 0) {
				JobHandle.CompleteAll(this.handles);
				this.handles.Dispose();
			}

			int managerCount = this.managers.Count;
			if (managerCount == 0) {
				this.scheduled = false;
				return;
			}

			// NOTE: Completeを複数回呼ぶとオーバーヘッドが高い
			//       …とはいえ一時バッファを毎フレーム扱うことのリスクは？
			this.handles = new NativeArray<JobHandle>(this.managers.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
			for (int i = 0; i < this.managers.Count; ++i)
				this.handles[i] = this.managers[i].ScheduleJob();

			JobHandle.ScheduleBatchedJobs();
		}

		/// <summary>
		/// 接続
		/// </summary>
		public static void Entry(SpringIJobManager manager) {
			// NOTE: 暗黙で準備するのは製品にはあまり良いと言えない
			if (instance == null) {
				GameObject go = new GameObject("SpringBoneJobScheduler");
				go.hideFlags |= HideFlags.HideInHierarchy;
				instance = go.AddComponent<SpringJobScheduler>();
			}
			
			instance.scheduled = true;
			instance.managers.Add(manager);
		}

		/// <summary>
		/// 切断
		/// </summary>
		public static bool Exit(SpringIJobManager manager) {
			if (instance == null)
				return false;

			return instance.managers.Remove(manager);
		}
	}
}
