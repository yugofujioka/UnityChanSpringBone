using UnityEngine;
using UnityEngine.Jobs;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Animations.SpringBones.Jobs {
	/// <summary>
	/// SpringBone計算の反映
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringBoneApplyJob : IJobParallelForTransform {
		[ReadOnly] public NativeArray<SpringBoneComponents> components;

		void IJobParallelForTransform.Execute(int index, TransformAccess transform) {
			// Apply
			transform.localRotation = this.components[index].localRotation;
		}
	}

	/// <summary>
	/// 親ノードの情報更新
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringParentJob : IJobParallelForTransform {
		[ReadOnly] public NativeArray<SpringBoneProperties> properties;

		[WriteOnly] public NativeArray<float4x4> components;

		void IJobParallelForTransform.Execute(int index, TransformAccess transform) {
			// NOTE: SpringBoneに依存する場合はtransform不要なのでキックした方が速い
			SpringBoneProperties prop = this.properties[index];
			if (prop.parentIndex < 0) {
				// NOTE: mathを使った方が速いが根っこのSpringBoneしか来ないので目立つ効果はない
				this.components[index] = new float4x4(transform.rotation, transform.position);
				//this.components[index] = transform.localToWorldMatrix;
			}
		}
	}

	/// <summary>
	/// Pivot位置の更新
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringPivotJob : IJobParallelForTransform {
		[ReadOnly] public NativeArray<SpringBoneProperties> properties;

		[WriteOnly] public NativeArray<float4x4> components;

		void IJobParallelForTransform.Execute(int index, TransformAccess transform) {
			// NOTE: SpringBoneに依存する場合はtransform不要なのでキックした方が速い
			SpringBoneProperties prop = this.properties[index];
			if (prop.pivotIndex < 0 && (prop.yAngleLimits.active || prop.zAngleLimits.active)) {
				// NOTE: mathを使った方が速いので、pivotがSpringBoneに依存していない場合が多いなら割と効果がある
				this.components[index] = new float4x4(transform.rotation, transform.position);
				//this.components[index] = transform.localToWorldMatrix;
			}

		}
	}

	/// <summary>
	/// コリジョンの更新
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringColliderJob : IJobParallelForTransform {
		[WriteOnly] public NativeArray<SpringColliderComponents> components;

		void IJobParallelForTransform.Execute(int index, TransformAccess transform) {
			// NOTE: mathで行列演算した方が速い
			float3 pos = transform.position;
			quaternion rot = transform.rotation;
			this.components[index] = new SpringColliderComponents {
				localToWorldMatrix = new float4x4(rot, pos),
				worldToLocalMatrix = math.mul(new float4x4(math.inverse(rot), float3.zero), float4x4.Translate(-pos)),
			};
			//this.components[index] = new SpringColliderComponents {
			//	localToWorldMatrix = transform.localToWorldMatrix,
			//	worldToLocalMatrix = transform.worldToLocalMatrix,
			//};
		}
	}

	/// <summary>
	/// 距離制限座標の更新
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringLengthTargetJob : IJobParallelForTransform {
		[WriteOnly] public NativeArray<float3> components;

		void IJobParallelForTransform.Execute(int index, TransformAccess transform) {
			this.components[index] = transform.position;
		}
	}
}
