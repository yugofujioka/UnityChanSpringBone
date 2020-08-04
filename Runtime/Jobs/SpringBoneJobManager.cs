
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Playables;
using UnityEngine.Animations;

#if !UNITY_2019_3_OR_NEWER
using UnityEngine.Experimental.Animations;
#endif

namespace Unity.Animations.SpringBones.Jobs
{
    public class SpringBoneJobManager : MonoBehaviour
    {
        [Header("Debug")] 
        public bool allowInspectorEdit = false;

        [Header("Animator")] 
        [Range(0f, 1f)] public float animatorBlendWeight = 1f;
        public bool blendAdditive;
        
        [Header("Properties")] 
        public bool isPaused = false;
        public int simulationFrameRate = 60;
        [Range(0f, 1f)] public float dynamicRatio = 0.5f;
        public Vector3 gravity = new Vector3(0f, -10f, 0f);
        [Range(0f, 1f)] public float bounce = 0f;
        [Range(0f, 1f)] public float friction = 1f;
        
        [Header("Constraints")] 
        public bool enableAngleLimits = true;
        public bool enableCollision = true;
        public bool enableLengthLimits = true;

        [Header("Ground Collision")] 
        public bool collideWithGround = true;
        public float groundHeight = 0f;

        private PlayableGraph m_Graph;
        private AnimationScriptPlayable m_SpringBonePlayable;
        private NativeArray<TransformStreamHandle> m_springBoneParentTransformHandles;
        private NativeArray<TransformStreamHandle> m_springBoneTransformHandles;
        private NativeArray<TransformStreamHandle> m_springColliderTransformHandles;
        private NativeArray<SpringBoneProperties> m_SpringBoneProperties;
        private NativeArray<SpringBoneComponent> m_springBoneComponents;
        private NativeArray<SpringColliderComponent> m_colliders; //read only
        private NativeArray<SpringColliderTransform> m_colliderTransforms;

        private SpringCollider[] m_colliderObjects;
        private SpringBone[] m_springBoneObjects;

       // private AnimationLayerMixerPlayable m_layerMixer;
        
        private PlayableGraph InitializeGraph()
        {
            m_springBoneObjects = FindSpringBones(true);
            var animator = GetComponent<Animator>();
            
            InitializeSpringBones(animator);
            InitializeColliders(animator);

            var springBoneJob = new SpringBoneJob
            {
                rootHandle = animator.BindStreamTransform(transform),
                springBoneTransformHandles = m_springBoneTransformHandles,
                springBoneParentTransformHandles = m_springBoneParentTransformHandles,
                springBoneProperties = m_SpringBoneProperties,
                springBoneComponents = m_springBoneComponents,
                springColliderTransformHandles = m_springColliderTransformHandles,
                forces = SpringBoneForceManager.GetManager().Forces,
                forceCount = SpringBoneForceManager.GetManager().ActiveForceCount,
                colliders = m_colliders,
                colliderTransforms = m_colliderTransforms,
                isPaused = isPaused,
                simulationFrameRate = simulationFrameRate,
                dynamicRatio = dynamicRatio,
                gravity = gravity,
                bounce = bounce,
                friction = friction,
                enableAngleLimits = enableAngleLimits,
                enableCollision = enableCollision,
                enableLengthLimits = enableLengthLimits,
                collideWithGround = collideWithGround,
                groundHeight = groundHeight,
            };

#if false
            // Create graph.
            var graph = PlayableGraph.Create("SpringBone");
            graph.SetTimeUpdateMode(DirectorUpdateMode.GameTime);

            m_SpringBonePlayable = AnimationScriptPlayable.Create(graph, springBoneJob);
            var output = AnimationPlayableOutput.Create(graph, "SpringBoneOutput", animator);

            // getting original animator moves
            var ctrlPlayable = AnimatorControllerPlayable.Create(graph, animator.runtimeAnimatorController);

            m_layerMixer = AnimationLayerMixerPlayable.Create(graph, 2);
            m_layerMixer.ConnectInput(0, ctrlPlayable, 0);
            m_layerMixer.ConnectInput(1, m_SpringBonePlayable, 0);
            m_layerMixer.SetLayerAdditive(1, blendAdditive);
            m_layerMixer.SetInputWeight(0, 1f - animatorBlendWeight);
            m_layerMixer.SetInputWeight(1, animatorBlendWeight);

            output.SetSourcePlayable(m_layerMixer);
#else
            AnimatorUtility.OptimizeTransformHierarchy(this.gameObject, null);

            var graph = animator.playableGraph;
            m_SpringBonePlayable = AnimationScriptPlayable.Create(graph, springBoneJob, 1);
            var output = AnimationPlayableOutput.Create(graph, "SpringBoneOutput", animator);
            var ctrlPlayable = AnimatorControllerPlayable.Create(graph, animator.runtimeAnimatorController);
            output.SetSourcePlayable(m_SpringBonePlayable);
            graph.Connect(ctrlPlayable, 0, m_SpringBonePlayable, 0);
#endif

            return graph;
        }
        
        private void InitializeColliders(Animator animator)
        {
            m_colliderObjects = FindSpringColliders();
            var nColliders = m_colliderObjects.Length;
            
            m_springColliderTransformHandles = new NativeArray<TransformStreamHandle>(nColliders, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
            
            m_colliders = new NativeArray<SpringColliderComponent>(nColliders, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);

            m_colliderTransforms = new NativeArray<SpringColliderTransform>(nColliders, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);

            for (var i = 0; i < nColliders; ++i)
            {
                m_springColliderTransformHandles[i] = animator.BindStreamTransform(m_colliderObjects[i].transform);
            }

            UpdateColliders();
        }
        
        private void InitializeSpringBones(Animator animator)
        {
            var nSpringBones = m_springBoneObjects.Length;
            m_springBoneParentTransformHandles = new NativeArray<TransformStreamHandle>(nSpringBones, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);

            m_springBoneTransformHandles = new NativeArray<TransformStreamHandle>(nSpringBones, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);

            m_SpringBoneProperties = new NativeArray<SpringBoneProperties>(nSpringBones, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
            
            m_springBoneComponents = new NativeArray<SpringBoneComponent>(nSpringBones, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
            
            for (var i = 0; i < nSpringBones; ++i)
            {
                m_springBoneTransformHandles[i] = animator.BindStreamTransform(m_springBoneObjects[i].transform);
                m_springBoneParentTransformHandles[i] = animator.BindStreamTransform(m_springBoneObjects[i].transform.parent);
            }

            for (var i = 0; i < nSpringBones; ++i)
            {
                InitializeSpringBoneComponent(i, in m_springBoneObjects[i]);
            }
        }
        
        // This should be called by the SpringManager in its Awake function before any updates
        private void InitializeSpringBoneComponent(int index, in SpringBone springBone)
        {
            var boneTransform = springBone.transform;
            var childPosition = ComputeChildBonePosition(springBone);
            var localChildPosition = boneTransform.InverseTransformPoint(childPosition);
            var boneAxis = localChildPosition.normalized;

            var initialLocalRotation = boneTransform.localRotation;
            var actualLocalRotation = initialLocalRotation;
            
            var springLength = Vector3.Distance(boneTransform.position, childPosition);
            var currTipPos = childPosition;
            var prevTipPos = childPosition;

            m_SpringBoneProperties[index] = new SpringBoneProperties
            {
                stiffnessForce = springBone.stiffnessForce,
                dragForce =  springBone.dragForce,
                springForce = springBone.springForce,
                windInfluence = springBone.windInfluence,
                angularStiffness = springBone.angularStiffness,
                yAngleLimits =  new AngleLimitComponent
                {
                    active = springBone.yAngleLimits.active ? 1 : 0,
                    min = springBone.yAngleLimits.min,
                    max = springBone.yAngleLimits.max,
                },
                zAngleLimits = new AngleLimitComponent
                {
                    active = springBone.zAngleLimits.active ? 1 : 0,
                    min = springBone.zAngleLimits.min,
                    max = springBone.zAngleLimits.max,
                },
                radius = springBone.radius,
                boneAxis = boneAxis,
                springLength = springLength,
                collisionMask = springBone.collisionMask
            };
            
            m_springBoneComponents[index] = new SpringBoneComponent
            {
                skinAnimationLocalRotation = initialLocalRotation, //temporal
                initialLocalRotation = initialLocalRotation,
                actualLocalRotation = actualLocalRotation,
                currentTipPosition = currTipPos,
                previousTipPosition = prevTipPos
            };
            
            // turn off SpringBone component to let Job work
            springBone.enabled = false; 
        }
        
        private void UpdateColliders()
        {
            for (var i = 0; i < m_colliderObjects.Length; ++i)
            {
                m_colliders[i] = new SpringColliderComponent
                {
                    layer = m_colliderObjects[i].layer,
                    type = m_colliderObjects[i].type,
                    radius = m_colliderObjects[i].radius,
                    width = m_colliderObjects[i].width,
                    height = m_colliderObjects[i].height,
                };
            }
        }

        private void UpdateSpringBoneProperties()
        {
            for (var i = 0; i < m_springBoneObjects.Length; ++i)
            {
                var springBone = m_springBoneObjects[i];
                m_SpringBoneProperties[i] = new SpringBoneProperties
                {
                    stiffnessForce = springBone.stiffnessForce,
                    dragForce =  springBone.dragForce,
                    springForce = springBone.springForce,
                    windInfluence = springBone.windInfluence,
                    angularStiffness = springBone.angularStiffness,
                    yAngleLimits =  new AngleLimitComponent
                    {
                        active = springBone.yAngleLimits.active ? 1 : 0,
                        min = springBone.yAngleLimits.min,
                        max = springBone.yAngleLimits.max,
                    },
                    zAngleLimits = new AngleLimitComponent
                    {
                        active = springBone.zAngleLimits.active ? 1 : 0,
                        min = springBone.zAngleLimits.min,
                        max = springBone.zAngleLimits.max,
                    },
                    radius = springBone.radius,
                    boneAxis = m_SpringBoneProperties[i].boneAxis,
                    springLength = m_SpringBoneProperties[i].springLength,
                    collisionMask = springBone.collisionMask
                };                
            }
        }

        private static Vector3 ComputeChildBonePosition(SpringBone bone)
        {
            var boneTransform = bone.transform;
            var children = GetValidSpringBoneChildren(boneTransform);
            var childCount = children.Count;

            if (childCount == 0)
            {
                // This should never happen
                Debug.LogWarning("SpringBone「" + bone.name + "」に有効な子供がありません");
                return bone.transform.position + boneTransform.right * -0.1f;
            }
            if (childCount == 1)
            {
                return children[0].position;
            }

            var initialTailPosition = new Vector3(0f, 0f, 0f);
            var averageDistance = 0f;
            var selfPosition = bone.transform.position;
            for (var childIndex = 0; childIndex < childCount; childIndex++)
            {
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
        
        private static IList<Transform> GetValidSpringBoneChildren(Transform parent)
        {
            // Ignore SpringBonePivots
            var childCount = parent.childCount;
            var children = new List<Transform>(childCount);
            for (int childIndex = 0; childIndex < childCount; childIndex++)
            {
                var child = parent.GetChild(childIndex);
                if (child.GetComponent<SpringBonePivot>() == null)
                {
                    children.Add(child);
                }
            }

            return children;
        }
        
        // Find SpringBones in children and assign them in depth order.
        // Note that the original list will be overwritten.
        private SpringBone[] FindSpringBones(bool includeInactive = false)
        {
            var unsortedSpringBones = GetComponentsInChildren<SpringBone>(includeInactive);
            var boneDepthList = unsortedSpringBones
                .Select(bone => new {bone, depth = GetObjectDepth(bone.transform)})
                .ToList();
            boneDepthList.Sort((a, b) => a.depth.CompareTo(b.depth));
            return boneDepthList.Select(item => item.bone).ToArray();
        }
       
        private SpringCollider[] FindSpringColliders()
        {
            return GetComponentsInChildren<SpringCollider>(false);
        }        

        // Get the depth of an object (number of consecutive parents)
        private static int GetObjectDepth(Transform inObject)
        {
            var depth = 0;
            var currentObject = inObject;
            while (currentObject != null)
            {
                currentObject = currentObject.parent;
                ++depth;
            }

            return depth;
        }        
        
        private void FinalizeGraph()
        {
            m_springBoneParentTransformHandles.Dispose();
            m_springBoneTransformHandles.Dispose();
            m_SpringBoneProperties.Dispose();
            m_springBoneComponents.Dispose();

            m_springColliderTransformHandles.Dispose();
            m_colliders.Dispose();
            m_colliderTransforms.Dispose();
            
            m_Graph.Destroy();
        }

        void OnEnable()
        {
            m_Graph = InitializeGraph();
            m_Graph.Play();
        }
        
        void OnDisable()
        {
            if (!m_Graph.IsValid())
            {
                return;
            }

            FinalizeGraph();
        }

        private void LateUpdate()
        {
            if (Application.isEditor && allowInspectorEdit)
            {
                UpdateSpringBoneProperties();
                UpdateColliders();
            }
            
            //m_layerMixer.SetInputWeight (0, 1f-animatorBlendWeight);
            //m_layerMixer.SetInputWeight (1, animatorBlendWeight);
            //m_layerMixer.SetLayerAdditive (1, blendAdditive);
            
            var jobData = m_SpringBonePlayable.GetJobData<SpringBoneJob>();

            jobData.isPaused = isPaused;
            jobData.simulationFrameRate = simulationFrameRate;
            jobData.dynamicRatio = dynamicRatio;
            jobData.gravity = gravity;
            jobData.bounce = bounce;
            jobData.friction = friction;
            jobData.enableAngleLimits = enableAngleLimits;
            jobData.enableCollision = enableCollision;
            jobData.enableLengthLimits = enableLengthLimits;
            jobData.collideWithGround = collideWithGround;
            jobData.groundHeight = groundHeight;
            jobData.forces = SpringBoneForceManager.GetManager().Forces;
            jobData.forceCount = SpringBoneForceManager.GetManager().ActiveForceCount;

            m_SpringBonePlayable.SetJobData(jobData);
        }
    }
}