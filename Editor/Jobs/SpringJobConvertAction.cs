using UnityEngine;
using UnityEditor;
using System.Collections.Generic;


namespace Unity.Animations.SpringBones.Jobs {
	public static class SpringJobConvertAction {
		[MenuItem("UTJ/選択したオブジェクトのSpringBoneをJob化")]
		public static void SwitchSpringJob() {
			if (EditorApplication.isPlaying || Application.isPlaying && EditorApplication.isCompiling)
				return;

			if (EditorUtility.DisplayDialog("Attention Please !", "選択したオブジェクト以下のSpringBoneをJob化します。\nSpringBoneは元に戻すまで編集が出来ません。\n\n実行してよろしいですか？", "いいえ", "はい"))
				return;

			var gameObjects = Selection.gameObjects;

			foreach (var go in gameObjects) {
				var bones = go.GetComponentsInChildren<SpringBone>(true);
				var jobColliderList = new List<SpringCollider>(128);
				foreach (var bone in bones) {
					// 対応済
					if ((bone.hideFlags & HideFlags.NotEditable) > 0)
						continue;

					jobColliderList.Clear();

					// Colliderのコンバート
					foreach (var col in bone.capsuleColliders) {
						if (!col.TryGetComponent<SpringCollider>(out var jobCol)) {
							jobCol = col.gameObject.AddComponent<SpringCollider>();
							jobCol.type = ColliderType.Capsule;
							jobCol.radius = col.radius;
							jobCol.height = col.height;
						}
						jobColliderList.Add(jobCol);
					}
					foreach (var col in bone.sphereColliders) {
						if (!col.TryGetComponent<SpringCollider>(out var jobCol)) {
							jobCol = col.gameObject.AddComponent<SpringCollider>();
							jobCol.type = ColliderType.Sphere;
							jobCol.radius = col.radius;
						}
						jobColliderList.Add(jobCol);
					}
					foreach (var col in bone.panelColliders) {
						if (!col.TryGetComponent<SpringCollider>(out var jobCol)) {
							jobCol = col.gameObject.AddComponent<SpringCollider>();
							jobCol.type = ColliderType.Panel;
							jobCol.width = col.width;
							jobCol.height = col.height;
						}
						jobColliderList.Add(jobCol);
					}

					// 元に戻す為に保持する
					//// NOTE: SerializeFieldなのでnullにしてもゼロ配列が入る
					//bone.capsuleColliders = null;
					//bone.sphereColliders = null;
					//bone.panelColliders = null;

					// NOTE: SerializeFieldなので反映しないと実行時に消される
					var so = new SerializedObject(bone);
					so.FindProperty("enabledJobSystem").boolValue = true; // Job化したら編集不可にする為のフラグ
					var prop = so.FindProperty("jobColliders");
					prop.arraySize = jobColliderList.Count;
					for (int i = 0; i < jobColliderList.Count; ++i)
						prop.GetArrayElementAtIndex(i).objectReferenceValue = jobColliderList[i];
					so.SetIsDifferentCacheDirty(); // シーンセーブせずにCtrl+Dで複製した場合の対処
					so.ApplyModifiedProperties();  // SerializedFieldの反映
				}

				// 元に戻すために保持する
				//// LegacyColliderの削除
				//var sphere = go.GetComponentsInChildren<SpringSphereCollider>(true);
				//foreach (var s in sphere)
				//	Object.DestroyImmediate(s);
				//var capsule = go.GetComponentsInChildren<SpringCapsuleCollider>(true);
				//foreach (var s in capsule)
				//	Object.DestroyImmediate(s);
				//var panel = go.GetComponentsInChildren<SpringPanelCollider>(true);
				//foreach (var s in panel)
				//	Object.DestroyImmediate(s);

				// SpringManagerのコンバート
				var managers = go.GetComponentsInChildren<SpringManager>(true);
				foreach (var manager in managers) {
					if (!manager.gameObject.TryGetComponent<SpringJobManager>(out var jobManager)) {
						jobManager = manager.gameObject.AddComponent<SpringJobManager>();
					}
					//jobManager.dynamicRatio = manager.dynamicRatio;
					jobManager.dynamicRatio = 1f; // 従来版とJob版で取り扱いが異なる
					jobManager.gravity = manager.gravity;
					jobManager.bounce = manager.bounce;
					jobManager.friction = manager.friction;
					jobManager.enableAngleLimits = manager.enableAngleLimits;
					jobManager.enableCollision = manager.enableCollision;
					jobManager.enableLengthLimits = manager.enableLengthLimits;
					jobManager.collideWithGround = manager.collideWithGround;
					jobManager.groundHeight = manager.groundHeight;

					Object.DestroyImmediate(manager);

					jobManager.CachedJobParam();
					var so = new SerializedObject(jobManager.gameObject);
					so.SetIsDifferentCacheDirty(); // シーンセーブせずにCtrl+Dで複製した場合の対処
					so.ApplyModifiedProperties();  // SerializedFieldの反映
				}

				var scheduler = Object.FindObjectOfType<SpringJobScheduler>();
				if (scheduler == null) {
					var schedulerGo = new GameObject("SpringJobScheduler(Don't destroy)");
					schedulerGo.AddComponent<SpringJobScheduler>();
				}
			}
		}

		[MenuItem("UTJ/選択したオブジェクトのSpringBoneを元に戻す")]
		public static void SwitchSpring() {
			if (EditorApplication.isPlaying || Application.isPlaying && EditorApplication.isCompiling)
				return;

			var gameObjects = Selection.gameObjects;

			foreach (var go in gameObjects) {
				var jobColliderList = go.GetComponentsInChildren<SpringCollider>(true);
				foreach (var col in jobColliderList) {
					col.gameObject.hideFlags &= ~HideFlags.NotEditable;
					Object.DestroyImmediate(col);
				}

				var bones = go.GetComponentsInChildren<SpringBone>(true);
				foreach (var bone in bones) {
					bone.enabled = true;
					var so = new SerializedObject(bone);
					so.FindProperty("enabledJobSystem").boolValue = false;
					so.FindProperty("index").intValue = 0;
					so.FindProperty("jobColliders").arraySize = 0;
					so.FindProperty("validChildren").arraySize = 0;
					so.SetIsDifferentCacheDirty(); // シーンセーブせずにCtrl+Dで複製した場合の対処
					so.ApplyModifiedProperties();  // SerializedFieldの反映
				}

				// SpringManagerのコンバート
				var managers = go.GetComponentsInChildren<SpringJobManager>(true);
				foreach (var manager in managers) {
					var origin = manager.gameObject.AddComponent<SpringManager>();
					//jobManager.dynamicRatio = manager.dynamicRatio;
					origin.dynamicRatio = 0.5f; // 従来版とJob版で取り扱いが異なる
					origin.gravity = manager.gravity;
					origin.bounce = manager.bounce;
					origin.friction = manager.friction;
					origin.enableAngleLimits = manager.enableAngleLimits;
					origin.enableCollision = manager.enableCollision;
					origin.enableLengthLimits = manager.enableLengthLimits;
					origin.collideWithGround = manager.collideWithGround;
					origin.groundHeight = manager.groundHeight;
					origin.FindSpringBones();

					Object.DestroyImmediate(manager);

					var so = new SerializedObject(origin.gameObject);
					so.SetIsDifferentCacheDirty(); // シーンセーブせずにCtrl+Dで複製した場合の対処
					so.ApplyModifiedProperties();  // SerializedFieldの反映
				}
			}
		}
	}
}
