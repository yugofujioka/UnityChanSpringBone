using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.Jobs;
using UnityEngine.Playables;


namespace Unity.Animations.SpringBones.JobExtend {
	public class SpringIJobManager : MonoBehaviour {
        [Header("Initialize Settings")] 
		public bool optimizeTransform = false;
        [Header("Debug")] 
        public bool allowInspectorEdit = false;
		[Header("Properties")]
		public bool isPaused = false;
		public int simulationFrameRate = 60;
		[Range(0f, 1f)] public float dynamicRatio = 0.5f;
		public Vector3 gravity = new Vector3(0f, -10f, 0f);
		[Range(0f, 1f)] public float bounce = 0f;
		[Range(0f, 1f)] public float friction = 1f;

		[Header("Constraints")] public bool enableAngleLimits = true;
		public bool enableCollision = true;
		public bool enableLengthLimits = true;

		[Header("Ground Collision")] public bool collideWithGround = true;
		public float groundHeight = 0f;

		// NOTE: Unity-Chan SpringBoneV2の設定をそのまま継続させる
		private SpringBone[] springBones;

		// ジョブ渡しバッファ
		private NativeArray<SpringBoneProperties> properties;
		private NativeArray<SpringBoneComponent> components;
		private NativeArray<LengthLimitComponent> lengthLimits;
		private NativeArray<Jobs.SpringColliderComponent> colliders;
		private NativeArray<Jobs.SpringColliderTransform> colliderTransforms;

		private Animator animator;
		private AnimationScriptPlayable springBonePlayable;
		private NativeArray<TransformStreamHandle> boneParentTransformHandles;
		private NativeArray<TransformStreamHandle> boneTransformHandles;
		private NativeArray<TransformStreamHandle> bonePivotTransformHandles;
		private NativeArray<TransformStreamHandle> colliderTransformHandles;
		private NativeArray<TransformStreamHandle> lengthLimitTransformHandles;

		private bool initialized = false;
		private SpringBoneJob job;
		private JobHandle handle;

		/// <summary>
		/// 初期化
		/// </summary>
		private void Initialize() {
			if (this.initialized)
				return;

			this.initialized = true;
			this.springBones = GetComponentsInChildren<SpringBone>(true);

			var nSpringBones = this.springBones.Length;

			this.components = new NativeArray<SpringBoneComponent>(nSpringBones, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			this.properties = new NativeArray<SpringBoneProperties>(nSpringBones, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

			this.animator = this.GetComponent<Animator>();
			this.boneTransformHandles = new NativeArray<TransformStreamHandle>(nSpringBones, Allocator.Persistent,
				NativeArrayOptions.UninitializedMemory);
			this.boneParentTransformHandles = new NativeArray<TransformStreamHandle>(nSpringBones, Allocator.Persistent,
				NativeArrayOptions.UninitializedMemory);
			this.bonePivotTransformHandles = new NativeArray<TransformStreamHandle>(nSpringBones, Allocator.Persistent,
				NativeArrayOptions.UninitializedMemory);

			this.InitializeSpringBoneComponent();

			// Colliders
			var colliders = this.GetComponentsInChildren<Jobs.SpringCollider>(true);
			int nColliders = colliders.Length;
			this.colliderTransformHandles = new NativeArray<TransformStreamHandle>(nColliders, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			this.colliders = new NativeArray<Jobs.SpringColliderComponent>(nColliders, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			this.colliderTransforms = new NativeArray<Jobs.SpringColliderTransform>(nColliders, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			for (int i = 0; i < nColliders; ++i) {
				Transform tr = colliders[i].transform;
				var comp = new Jobs.SpringColliderComponent() {
                    layer = colliders[i].layer,
                    type = colliders[i].type,
                    radius = colliders[i].radius,
                    width = colliders[i].width,
                    height = colliders[i].height,
				};
				this.colliders[i] = comp;
				this.colliderTransformHandles[i] = this.animator.BindStreamTransform(tr);
				
				if (this.optimizeTransform)
					Object.DestroyImmediate(colliders[i]);
			}

			var settings = new SpringBoneSettings() {
				deltaTime = (this.simulationFrameRate > 0) ? (1f / this.simulationFrameRate) : Time.deltaTime,
				dynamicRatio = this.dynamicRatio,
				gravity = this.gravity,
				bounce = this.bounce,
				friction = this.friction,
				enableAngleLimits = this.enableAngleLimits,
				enableCollision = this.enableCollision,
				enableLengthLimits = this.enableLengthLimits,
				collideWithGround = this.collideWithGround,
				groundHeight = this.groundHeight,
			};

			this.job = new SpringBoneJob() {
				settings = settings,
				properties = this.properties,
				lengthLimits = this.lengthLimits,
				colliders = this.colliders,
                colliderTransforms = this.colliderTransforms,

				components = this.components,
			};

			var springBoneJob = new SpringAnimationJob {
				//rootHandle = this.animator.BindStreamTransform(this.transform),
				boneTransformHandles = this.boneTransformHandles,
				boneParentTransformHandles = this.boneParentTransformHandles,
				bonePivotTransformHandles = this.bonePivotTransformHandles,
				colliderTransforms = this.colliderTransforms,
				colliderTransformHandles = this.colliderTransformHandles,

				settings = settings,
				properties = this.properties,
				components = this.components,
			};

			// Transformの階層構造をバラす
			if (this.optimizeTransform) {
				AnimatorUtility.OptimizeTransformHierarchy(this.gameObject, null);
				this.springBones = null;
			}

			var graph = this.animator.playableGraph;
			this.springBonePlayable = AnimationScriptPlayable.Create(graph, springBoneJob, 1);
			var output = AnimationPlayableOutput.Create(graph, "SpringBoneOutput", animator);
			var ctrlPlayable = AnimatorControllerPlayable.Create(graph, animator.runtimeAnimatorController);
			output.SetSourcePlayable(this.springBonePlayable);
			graph.Connect(ctrlPlayable, 0, this.springBonePlayable, 0);
			graph.Play();
		}

		/// <summary>
		/// 破棄
		/// </summary>
		private void Final() {
			this.handle.Complete();

			this.properties.Dispose();
			this.components.Dispose();

			this.lengthLimits.Dispose();
			this.colliders.Dispose();
			this.colliderTransforms.Dispose();

			this.colliderTransformHandles.Dispose();
			this.boneParentTransformHandles.Dispose();
			this.boneTransformHandles.Dispose();
			this.bonePivotTransformHandles.Dispose();
			this.lengthLimitTransformHandles.Dispose();
			
			//this.animator.playableGraph.Destroy();
			this.animator.playableGraph.Stop();
			this.animator.playableGraph.Disconnect(this.springBonePlayable, 0);
			this.springBonePlayable.Destroy();
		}

		// This should be called by the SpringManager in its Awake function before any updates
		private void InitializeSpringBoneComponent() {
			
			List<Transform> lengthTargetList = new List<Transform>(256);
			List<LengthLimitComponent> lengthLimitList = new List<LengthLimitComponent>(256);

			for (var i = 0; i < this.springBones.Length; ++i) {
				SpringBone springBone = this.springBones[i];
				springBone.index = i;

				var root = springBone.transform;
				var parent = root.parent;

				var childPos = ComputeChildBonePosition(springBone);
				var childLocalPos = root.InverseTransformPoint(childPos);
				var boneAxis = Vector3.Normalize(childLocalPos);

				var worldPos = root.position;
				var worldRot = root.rotation;

				var springLength = Vector3.Distance(worldPos, childPos);
				var currTipPos = childPos;
				var prevTipPos = childPos;

				// Length Limit
				var targetIndex = lengthTargetList.Count;
				var targetCount = springBone.lengthLimitTargets.Length;
				//lengthsToLimitTargets = new float[targetCount];
				for (int target = 0; target < targetCount; ++target) {
					Transform targetRoot = springBone.lengthLimitTargets[target].transform;
					lengthTargetList.Add(targetRoot);
					var lengthLimit = new LengthLimitComponent {
						position = targetRoot.position,
						target = Vector3.Magnitude(targetRoot.position - childPos),
					};
					lengthLimitList.Add(lengthLimit);
				}

				// ReadOnly
				int parentIndex = -1;
				if (parent.TryGetComponent<SpringBone>(out var parentBone))
					parentIndex = parentBone.index;
				this.properties[i] = new SpringBoneProperties {
					stiffnessForce = springBone.stiffnessForce,
					dragForce = springBone.dragForce,
					springForce = springBone.springForce,
					windInfluence = springBone.windInfluence,
					angularStiffness = springBone.angularStiffness,
					yAngleLimits = new Jobs.AngleLimitComponent {
						active = springBone.yAngleLimits.active ? 1 : 0,
						min = springBone.yAngleLimits.min,
						max = springBone.yAngleLimits.max,
					},
					zAngleLimits = new Jobs.AngleLimitComponent {
						active = springBone.zAngleLimits.active ? 1 : 0,
						min = springBone.zAngleLimits.min,
						max = springBone.zAngleLimits.max,
					},
					radius = springBone.radius,
					boneAxis = boneAxis,
					springLength = springLength,
                    collisionMask = springBone.collisionMask,

					lengthLimitIndex = targetIndex,
					lengthLimitLength = targetCount,
					
					parentIndex = parentIndex,
					localPosition = root.localPosition,
					initialLocalRotation = root.localRotation,
				};

				// Read/Write
				this.components[i] = new SpringBoneComponent {
					currentTipPosition = currTipPos,
					previousTipPosition = prevTipPos,
					localRotation = root.localRotation,
				};

				this.boneParentTransformHandles[i] = this.animator.BindStreamTransform(root.parent);
				this.boneTransformHandles[i] = this.animator.BindStreamTransform(root);
				this.bonePivotTransformHandles[i] = this.animator.BindStreamTransform(springBone.GetPivotTransform());

				// turn off SpringBone component to let Job work
				springBone.enabled = false;

				if (this.optimizeTransform)
					Object.DestroyImmediate(springBone);
			}
				
			// LengthLimit
			// NOTE: Inspector拡張で静的にバッファ用意した方がベター
			int nLengthLimits = this.lengthLimits.Length;
			this.lengthLimitTransformHandles = new NativeArray<TransformStreamHandle>(this.lengthLimits.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			for (int n = 0; n < nLengthLimits; ++n)
				this.lengthLimitTransformHandles[n] = this.animator.BindStreamTransform(lengthTargetList[n]);
			this.lengthLimits = new NativeArray<LengthLimitComponent>(lengthLimitList.ToArray(), Allocator.Persistent);
		}

		private static Vector3 ComputeChildBonePosition(SpringBone bone) {
			var children = GetValidSpringBoneChildren(bone.transform);
			var childCount = children.Count;

			if (childCount == 0) {
				// This should never happen
				Debug.LogWarning("SpringBone「" + bone.name + "」に有効な子供がありません");
				return bone.transform.position + bone.transform.right * -0.1f;
			}

			if (childCount == 1) {
				return children[0].position;
			}

			var initialTailPosition = new Vector3(0f, 0f, 0f);
			var averageDistance = 0f;
			var selfPosition = bone.transform.position;
			for (int childIndex = 0; childIndex < childCount; childIndex++) {
				var childPosition = children[childIndex].position;
				initialTailPosition += childPosition;
				averageDistance += (childPosition - selfPosition).magnitude;
			}

			averageDistance /= childCount;
			initialTailPosition /= childCount;
			var selfToInitial = initialTailPosition - selfPosition;
			selfToInitial.Normalize();
			initialTailPosition = selfPosition + averageDistance * selfToInitial;
			return initialTailPosition;
		}

		private static IList<Transform> GetValidSpringBoneChildren(Transform parent) {
			// Ignore SpringBonePivots
			var childCount = parent.childCount;
			var children = new List<Transform>(childCount);
			for (int childIndex = 0; childIndex < childCount; childIndex++) {
				var child = parent.GetChild(childIndex);
				if (child.GetComponent<SpringBonePivot>() == null) {
					children.Add(child);
				}
			}

			return children;
		}

		/// <summary>
		/// 演算ジョブ発行
		/// </summary>
		public JobHandle ScheduleJob() {
			//this.handle.Complete();

			if (this.isPaused)
				return this.handle;

#if UNITY_EDITOR
			if (this.allowInspectorEdit && !this.optimizeTransform) {
				this.job.settings.deltaTime = (this.simulationFrameRate > 0) ? (1f / this.simulationFrameRate) : Time.deltaTime;
				this.job.settings.dynamicRatio = this.dynamicRatio;
				this.job.settings.gravity = this.gravity;
				this.job.settings.bounce = this.bounce;
				this.job.settings.friction = this.friction;
				this.job.settings.enableAngleLimits = this.enableAngleLimits;
				this.job.settings.enableCollision = this.enableCollision;
				this.job.settings.enableLengthLimits = this.enableLengthLimits;
				this.job.settings.collideWithGround = this.collideWithGround;
				this.job.settings.groundHeight = this.groundHeight;

				for (int i = 0; i < this.springBones.Length; ++i) {
					var springBone = this.springBones[i];
					var prop = this.properties[i];
					prop.stiffnessForce = springBone.stiffnessForce;
					prop.dragForce = springBone.dragForce;
					prop.springForce = springBone.springForce;
					prop.windInfluence = springBone.windInfluence;
					prop.angularStiffness = springBone.angularStiffness;
					prop.yAngleLimits = new Jobs.AngleLimitComponent {
						active = springBone.yAngleLimits.active ? 1 : 0,
						min = springBone.yAngleLimits.min,
						max = springBone.yAngleLimits.max,
					};
					prop.zAngleLimits = new Jobs.AngleLimitComponent {
						active = springBone.zAngleLimits.active ? 1 : 0,
						min = springBone.zAngleLimits.min,
						max = springBone.zAngleLimits.max,
					};
					prop.radius = springBone.radius;

					this.properties[i] = prop;
				}
			}
#endif
			
			this.handle = this.job.Schedule();

			return this.handle;
		}

		void OnEnable() {
			this.Initialize();
			SpringJobScheduler.Entry(this);
		}
		void OnDisable() {
			SpringJobScheduler.Exit(this);
		}
		private void OnDestroy() {
			this.Final();
		}
	}
}
