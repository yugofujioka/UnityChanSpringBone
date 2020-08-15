using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using FUtility;

namespace Unity.Animations.SpringBones.Jobs {
	/// <summary>
	/// ボーン毎の設定値（ReadOnly in Job）
	/// </summary>
	[System.Serializable]
	public struct SpringBoneProperties {
		public float stiffnessForce;
		public float dragForce;
		public float3 springForce;
		public float windInfluence;
		public float angularStiffness;
		public AngleLimitComponent yAngleLimits;
		public AngleLimitComponent zAngleLimits;
		public float radius;
		public float springLength;
		public float3 boneAxis;
		public float3 localPosition;
		public quaternion initialLocalRotation;
		public int parentIndex; // 親がSpringBoneだった場合のIndex（違う場合 -1）

		public int pivotIndex;  // PivotがSpringBoneだった場合のIndex（違う場合 -1）
		public float4x4 pivotLocalMatrix;

		public NestedNativeArray<int> collisionNumbers;
		public NestedNativeArray<LengthLimitProperties> lengthLimitProps;
	}

	/// <summary>
	/// 更新されるボーン毎の値（Read/Write in Job）
	/// </summary>
	[System.Serializable]
	public struct SpringBoneComponents {
		public float3 currentTipPosition;
		public float3 previousTipPosition;
		public quaternion localRotation;
		public float3 position;
		public quaternion rotation;
	}

	/// <summary>
	/// 距離制限の設定値
	/// </summary>
	[System.Serializable]
	public struct LengthLimitProperties {
		public int targetIndex; // targetがSpringBoneだった場合のIndex（違う場合 -1）
		public float target;
	}

	/// <summary>
	/// SpringBone計算ジョブ
	/// </summary>
	[Burst.BurstCompile]
	public struct SpringJob : IJobParallelFor {
		[ReadOnly] public NativeArray<SpringJobElement> jobManagers;
		[ReadOnly] public NativeArray<SpringForceComponent> forces;
		public int forceCount;

		/// <summary>
		/// ジョブ実行
		/// </summary>
		void IJobParallelFor.Execute(int index) {
			this.jobManagers[index].Execute(forces, forceCount);
		}
	}

	/// <summary>
	/// SpringManager単位の計算
	/// </summary>
	public struct SpringJobElement {
		// JobManager settings
		public bool isPaused;
		public float deltaTime;
		public float dynamicRatio;
		public float3 gravity;
		public float bounce;
		public float friction;
		public bool enableAngleLimits;
		public bool enableCollision;
		public bool enableLengthLimits;
		public bool collideWithGround;
		public float groundHeight;

		public NestedNativeArray<SpringBoneProperties> nestedProperties;
		public NestedNativeArray<SpringBoneComponents> nestedComponents;
		public NestedNativeArray<float4x4> nestedParentComponents;
		public NestedNativeArray<float4x4> nestedPivotComponents;
		public NestedNativeArray<SpringColliderProperties> nestedColliderProperties;
		public NestedNativeArray<SpringColliderComponents> nestedColliderComponents;
		public NestedNativeArray<float3> nestedLengthLimitTargets;

		/// <summary>
		/// ジョブ実行
		/// </summary>
		public void Execute(NativeArray<SpringForceComponent> forces, int forceCount) {
			if (this.isPaused)
				return;

			var length = this.nestedProperties.Length;
			for (int i = 0; i < length; ++i) {
				var bone = this.nestedComponents[i];
				var prop = this.nestedProperties[i];

				quaternion parentRot;
				if (prop.parentIndex >= 0) {
					// 親ノードがSpringBoneなら演算結果を反映する
					var parentBone = this.nestedComponents[prop.parentIndex];
					parentRot = parentBone.rotation;
					bone.position = parentBone.position + math.mul(parentBone.rotation, prop.localPosition);
					bone.rotation = math.mul(parentRot, bone.localRotation);
				} else {
					var parentMat = this.nestedParentComponents[i];
					parentRot = new quaternion(parentMat);
					
					bone.position = math.transform(parentMat, prop.localPosition);
					bone.rotation = math.mul(parentRot, bone.localRotation);
				}

				var baseWorldRotation = math.mul(parentRot, prop.initialLocalRotation);
				var force = this.GetTotalForceOnBone(in bone, in prop, forces, forceCount);
				this.UpdateSpring(ref bone, in prop, baseWorldRotation, force);
				this.ResolveCollisionsAndConstraints(ref bone, in prop, i, this.nestedComponents);
				this.UpdateRotation(ref bone, in prop, baseWorldRotation);
				
				// NOTE: 子の為に更新する
				bone.rotation = math.mul(parentRot, bone.localRotation);

				this.nestedComponents[i] = bone;
			}
		}

		private void UpdateSpring(ref SpringBoneComponents bone, in SpringBoneProperties prop, quaternion baseWorldRotation, float3 externalForce) {
			var orientedInitialPosition = bone.position + math.mul(baseWorldRotation, prop.boneAxis) * prop.springLength;

			// Hooke's law: force to push us to equilibrium
			var force = prop.stiffnessForce * (orientedInitialPosition - bone.currentTipPosition);
			force += prop.springForce + externalForce;
			var sqrDt = this.deltaTime * this.deltaTime;
			force *= 0.5f * sqrDt;

			var temp = bone.currentTipPosition;
			force += (1f - prop.dragForce) * (bone.currentTipPosition - bone.previousTipPosition);
			bone.currentTipPosition += force;
			bone.previousTipPosition = temp;

			// Inlined because FixBoneLength is slow
			var headPosition = bone.position;
			var headToTail = bone.currentTipPosition - headPosition;
			var magnitude = math.length(headToTail);

			const float MagnitudeThreshold = 0.001f;
			if (magnitude <= MagnitudeThreshold) {
				var mat = new float4x4(bone.rotation, bone.position);
				headToTail = mat.MultiplyVector(prop.boneAxis);
			} else {
				headToTail /= magnitude;
			}

			bone.currentTipPosition = headPosition + prop.springLength * headToTail;
		}

		private void ResolveCollisionsAndConstraints(ref SpringBoneComponents bone, in SpringBoneProperties prop, int index, NestedNativeArray<SpringBoneComponents> boneComponents) {
			if (this.enableLengthLimits)
				this.ApplyLengthLimits(ref bone, in prop, boneComponents);

			var hadCollision = false;

			if (this.collideWithGround)
				hadCollision = this.ResolveGroundCollision(ref bone, in prop);

			if (this.enableCollision && !hadCollision)
				this.ResolveCollisions(ref bone, in prop);

			if (this.enableAngleLimits) {
				float4x4 pivotLocalToWorld;
				if (prop.pivotIndex >= 0) {
					var pivotBone = boneComponents[prop.pivotIndex];
					pivotLocalToWorld = math.mul(new float4x4(pivotBone.rotation, pivotBone.position), prop.pivotLocalMatrix);
				} else {
					pivotLocalToWorld = this.nestedPivotComponents[index];
				}
				this.ApplyAngleLimits(ref bone, in prop, in pivotLocalToWorld);
			}
		}

		// Returns the new tip position
		private void ApplyLengthLimits(ref SpringBoneComponents bone, in SpringBoneProperties prop, NestedNativeArray<SpringBoneComponents> boneComponents) {
			var length = prop.lengthLimitProps.Length;
			if (length == 0)
				return;

			const float SpringConstant = 0.5f;
			var accelerationMultiplier = SpringConstant * this.deltaTime * this.deltaTime;
			var movement = float3.zero;
			for (int i = 0; i < length; ++i) {
				var limit = prop.lengthLimitProps[i];
				var lengthToLimitTarget = limit.target;
				// TODO: pivotの時と同様にLengthLimitノードがSpringBoneの下についていた場合は反映が遅れてる、そのようなケースがある？
				var limitPosition = (limit.targetIndex >= 0) ? boneComponents[limit.targetIndex].position
															 : this.nestedLengthLimitTargets[i];
				var currentToTarget = bone.currentTipPosition - limitPosition;
				var currentDistanceSquared = math.dot(currentToTarget, currentToTarget);

				// Hooke's Law
				var currentDistance = math.sqrt(currentDistanceSquared);
				var distanceFromEquilibrium = currentDistance - lengthToLimitTarget;
				movement -= accelerationMultiplier * distanceFromEquilibrium * math.normalize(currentToTarget);
			}

			bone.currentTipPosition += movement;
		}

		private bool ResolveGroundCollision(ref SpringBoneComponents bone, in SpringBoneProperties prop) {
			var groundHeight = this.groundHeight;
			// Todo: this assumes a flat ground parallel to the xz plane
			var worldHeadPosition = bone.position;
			var worldTailPosition = bone.currentTipPosition;
			// NOTE: スケールが反映されないのだからスカラー値が欲しいなら行列演算する意味はないのでは…？
			var worldRadius = prop.radius;//transform.TransformDirection(prop.radius, 0f, 0f).magnitude;
			var worldLength = math.length(bone.currentTipPosition - worldHeadPosition);
			worldHeadPosition.y -= groundHeight;
			worldTailPosition.y -= groundHeight;

			var collidingWithGround = SpringCollisionResolver.ResolvePanelOnAxis(
				worldHeadPosition, ref worldTailPosition, worldLength, worldRadius, SpringCollisionResolver.Axis.Y);

			if (collidingWithGround) {
				worldTailPosition.y += groundHeight;
				bone.currentTipPosition = FixBoneLength(ref bone, in prop, in worldTailPosition);
				// Todo: bounce, friction
				bone.previousTipPosition = bone.currentTipPosition;
			}

			return collidingWithGround;
		}

		private static float3 FixBoneLength(ref SpringBoneComponents bone, in SpringBoneProperties prop, in float3 tailPosition) {
			var minLength = 0.5f * prop.springLength;
			var maxLength = prop.springLength;
			var headPosition = bone.position;
			var headToTail = tailPosition - headPosition;
			var magnitude = math.length(headToTail);

			const float MagnitudeThreshold = 0.001f;
			if (magnitude <= MagnitudeThreshold) {
				var mat = new float4x4(bone.rotation, bone.position);
				return headPosition + math.rotate(mat, prop.boneAxis) * minLength;
			}

			var newMagnitude = (magnitude < minLength) ? minLength : magnitude;
			newMagnitude = (newMagnitude > maxLength) ? maxLength : newMagnitude;
			return headPosition + (newMagnitude / magnitude) * headToTail;
		}

		private bool ResolveCollisions(ref SpringBoneComponents bone, in SpringBoneProperties prop) {
			var desiredPosition = bone.currentTipPosition;
			var headPosition = bone.position;

			// var scaledRadius = transform.TransformDirection(radius, 0f, 0f).magnitude;
			// var scaleMagnitude = new Vector3(prop.radius, 0f, 0f).magnitude;
			var hitNormal = new float3(0f, 0f, 1f);

			var hadCollision = false;

			var length = prop.collisionNumbers.Length;
			for (var i = 0; i < length; ++i) {
				var num = prop.collisionNumbers[i];
				var collider = this.nestedColliderProperties[num];
				var colliderTransform = this.nestedColliderComponents[num];

				switch (collider.type) {
					case ColliderType.Capsule:
						hadCollision |= SpringCollisionResolver.ResolveCapsule(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal,
							prop.radius);
						break;
					case ColliderType.Sphere:
						hadCollision |= SpringCollisionResolver.ResolveSphere(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal, prop.radius);
						break;
					case ColliderType.Panel:
						hadCollision |= SpringCollisionResolver.ResolvePanel(
							collider, colliderTransform,
							headPosition, ref bone.currentTipPosition, ref hitNormal, prop.springLength,
							prop.radius);
						break;
				}
			}

			if (hadCollision) {
				var incidentVector = desiredPosition - bone.previousTipPosition;
				var reflectedVector = math.reflect(incidentVector, hitNormal);

				// friction
				var upwardComponent = math.dot(reflectedVector, hitNormal) * hitNormal;
				var lateralComponent = reflectedVector - upwardComponent;

				var bounceVelocity = this.bounce * upwardComponent + (1f - this.friction) * lateralComponent;
				const float BounceThreshold = 0.0001f;
				if (math.dot(bounceVelocity, bounceVelocity) > BounceThreshold) {
					var distanceTraveled = math.length(bone.currentTipPosition - bone.previousTipPosition);
					bone.previousTipPosition = bone.currentTipPosition - bounceVelocity;
					bone.currentTipPosition += math.max(0f, math.length(bounceVelocity) - distanceTraveled) * math.normalize(bounceVelocity);
				} else {
					bone.previousTipPosition = bone.currentTipPosition;
				}
			}
			return hadCollision;
		}

		private void ApplyAngleLimits(ref SpringBoneComponents bone, in SpringBoneProperties prop, in float4x4 pivotLocalToWorld) {
			if (!prop.yAngleLimits.active && !prop.zAngleLimits.active)
				return;

			var origin = bone.position;
			var vector = bone.currentTipPosition - origin;

			var forward = math.transform(pivotLocalToWorld, new float3(-1f, 0f, 0f));// Vector3.right;

			var mulBack = math.transform(pivotLocalToWorld, new float3(0f, 0f, -1f));// Vector3.back;
			var mulDown = math.transform(pivotLocalToWorld, new float3(0f, -1f, 0f));// Vector3.down;

			if (prop.yAngleLimits.active) {
				vector = prop.yAngleLimits.ConstrainVector(
							vector,
							mulDown, //pivotLocalToWorld * -Vector3.up,
							mulBack, //pivotLocalToWorld * -Vector3.forward,
							forward, prop.angularStiffness, this.deltaTime);
			}

			if (prop.zAngleLimits.active) {
				vector = prop.zAngleLimits.ConstrainVector(
							vector,
							mulBack, //pivotLocalToWorld * -Vector3.forward,
							mulDown, //pivotLocalToWorld * -Vector3.up,
							forward, prop.angularStiffness, this.deltaTime);
			}

			bone.currentTipPosition = origin + vector;
		}

		private void UpdateRotation(ref SpringBoneComponents bone, in SpringBoneProperties prop, quaternion baseWorldRotation) {
			if (float.IsNaN(bone.currentTipPosition.x)
				| float.IsNaN(bone.currentTipPosition.y)
				| float.IsNaN(bone.currentTipPosition.z))
			{
				bone.currentTipPosition = bone.position + math.mul(baseWorldRotation, prop.boneAxis) * prop.springLength;
				bone.previousTipPosition = bone.currentTipPosition;
				Debug.LogError("SpringBone : currentTipPosition is NaN.");
			}

			var actualLocalRotation = ComputeLocalRotation(in baseWorldRotation, ref bone, in prop);
			bone.localRotation = math.nlerp(bone.localRotation, actualLocalRotation, this.dynamicRatio);
		}

		public static quaternion FromToRotation(float3 from, float3 to) {
			return quaternion.AxisAngle(
				math.normalize(math.cross(from, to)),
				math.acos(math.clamp(math.dot(math.normalize(from), math.normalize(to)), -1f, 1f)));
		}

		private static quaternion ComputeLocalRotation(in quaternion baseWorldRotation, ref SpringBoneComponents bone, in SpringBoneProperties prop) {
			var worldBoneVector = bone.currentTipPosition - bone.position;
			var localBoneVector = math.mul(math.inverse(baseWorldRotation), worldBoneVector);
			localBoneVector = math.normalize(localBoneVector);

			var aimRotation = FromToRotation(prop.boneAxis, localBoneVector);
			var outputRotation = math.mul(prop.initialLocalRotation, aimRotation);

			return outputRotation;
		}

		private float3 GetTotalForceOnBone(in SpringBoneComponents bone, in SpringBoneProperties prop, NativeArray<SpringForceComponent> forces, int forceCount) {
			var sumOfForces = this.gravity;
			for (var i = 0; i < forceCount; i++) {
				var force = forces[i];
				sumOfForces += ComputeForceOnBone(in force, in bone, prop.windInfluence);
			}

			return sumOfForces;
		}

		// ForceVolume
		private static float3 ComputeForceOnBone(in SpringForceComponent force, in SpringBoneComponents bone, float boneWindInfluence) {
			//Directional
			if (force.type == SpringBoneForceType.Directional)
				return math.rotate(new float4x4(force.rotation, force.position), new float3(0f, 0f, force.strength));

			//Wind
			//var fullWeight = force.weight * force.strength;
			var fullWeight = force.strength;
			//if ((fullWeight <= 0.0001f) | (force.periodInSecond <= 0.001f))
			if (fullWeight <= 0.0001f)
				return float3.zero;

			const float PI2 = math.PI * 2f;

			//var factor = force.timeInSecond / force.periodInSecond * PI2;

			var localToWorldMatrix = new float4x4(force.rotation, force.position);
			var worldToLocalMatrix = math.mul(new float4x4(math.inverse(force.rotation), float3.zero), float4x4.Translate(-force.position));

			// Wind
			var boneLocalPositionInWindWorld = math.transform(worldToLocalMatrix, bone.position);
			var positionalMultiplier = PI2 / force.peakDistance;
			var positionalFactor = math.sin(positionalMultiplier * boneLocalPositionInWindWorld.x) + math.cos(positionalMultiplier * boneLocalPositionInWindWorld.z);
			var offsetMultiplier = math.sin(force.timeFactor + positionalFactor);

			var forward = math.rotate(localToWorldMatrix, new float3(0f, 0f, 1f));
			float3 offset = offsetMultiplier * force.offsetVector;
			var forceAtPosition = boneWindInfluence * fullWeight * math.normalize(forward + offset);

			return forceAtPosition;
		}
	}
}
