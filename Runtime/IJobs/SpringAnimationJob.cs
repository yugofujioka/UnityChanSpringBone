using Unity.Collections;
using UnityEngine;
using UnityEngine.Animations;

namespace Unity.Animations.SpringBones.JobExtend {
	//[Burst.BurstCompile]
	public struct SpringAnimationJob : IAnimationJob {
		//[ReadOnly] public TransformStreamHandle rootHandle;
		[ReadOnly] public NativeArray<TransformStreamHandle> boneTransformHandles;
		[ReadOnly] public NativeArray<TransformStreamHandle> boneParentTransformHandles;
		[ReadOnly] public NativeArray<TransformStreamHandle> bonePivotTransformHandles;
		[ReadOnly] public NativeArray<TransformStreamHandle> colliderTransformHandles;
		[ReadOnly] public NativeArray<TransformStreamHandle> lengthLimitTransformHandles;
		[ReadOnly] public NativeArray<SpringBoneProperties> properties;
		[ReadOnly] public SpringBoneSettings settings;
		
		[WriteOnly] public NativeArray<Jobs.SpringColliderTransform> colliderTransforms;
		public NativeArray<SpringBoneComponent> components;

		public void ProcessRootMotion(AnimationStream stream) {
			//Vector3 rootPosition;
			//Quaternion rootRotation;
			//this.rootHandle.GetGlobalTR(stream, out rootPosition, out rootRotation);
			//this.rootHandle.SetGlobalTR(stream, rootPosition, rootRotation, true);
		}
		public void ProcessAnimation(AnimationStream stream) {
			Vector3 pos; Quaternion rot;
			int length = this.components.Length;
			if (length < 2)
				return;

			for (int i = 0; i < length; ++i) {
				SpringBoneComponent bone = this.components[i];
				SpringBoneProperties prop = this.properties[i];
				
				this.boneTransformHandles[i].SetLocalRotation(stream, bone.localRotation);

				// Parent
				if (prop.parentIndex < 0)
					this.boneParentTransformHandles[i].GetGlobalTR(stream, out bone.parentPosition, out bone.parentRotation);

				// Pivot
				if (this.settings.enableAngleLimits) {
					if (prop.yAngleLimits.active > 0 || prop.zAngleLimits.active > 0) {
						this.bonePivotTransformHandles[i].GetGlobalTR(stream, out pos, out rot);
						bone.pivotLocalToGlobalMat = Matrix4x4.TRS(pos, rot, Vector3.one);
					}
				}

				this.components[i] = bone;
			}

			// Collider
			if (this.settings.enableCollision) {
				int colliderLength = this.colliderTransformHandles.Length;
				for (var col = 0; col < colliderLength; ++col) {
					this.colliderTransformHandles[col].GetGlobalTR(stream, out pos, out rot);

					var mat = Matrix4x4.TRS(pos, rot, Vector3.one);
					this.colliderTransforms[col] = new Jobs.SpringColliderTransform {
						position = pos,
						rotation = rot,
						localToWorldMatrix = mat,
						worldToLocalMatrix = Matrix4x4.Inverse(mat),
					};
				}
			}
		}
	}
}