//
//  Transform.swift
//  TransformSwift
//
//  Created by Boris Gromov on 24/05/2019.
//  Copyright © 2019 Volaly. All rights reserved.


import simd

// Inspired by ROS tf library
public class Transform {
    public static let identity: Transform = Transform(simd_double3x3(1.0), simd_double3(repeating: 0.0))

    private var mBasis: simd_double3x3
    private var mOrigin: simd_double3

    public var basis: simd_double3x3 {
        get {
            return mBasis
        }
        set {
            mBasis = newValue
        }
    }
    public var rotation: simd_quatd {
        get {
            return simd_quatd(mBasis)
        }
        set {
            mBasis = simd_double3x3.init(newValue)
        }
    }
    public var origin: simd_double3 {
        get {
            return mOrigin
        }
        set {
            mOrigin = newValue
        }
    }
    public var inversed: Transform {
        let inv = mBasis.transpose
        return Transform(inv, inv * -mOrigin)
    }

    public var matrix: simd_double4x4 {
        let m = simd_double4x4(simd_double4(mBasis.columns.0.x, mBasis.columns.0.y, mBasis.columns.0.z, 0.0),
                               simd_double4(mBasis.columns.1.x, mBasis.columns.1.y, mBasis.columns.1.z, 0.0),
                               simd_double4(mBasis.columns.2.x, mBasis.columns.2.y, mBasis.columns.2.z, 0.0),
                               simd_double4(mOrigin.x,          mOrigin.y,          mOrigin.z,          1.0))
        return m
    }

    public init(_ matrix: simd_double4x4) {
        mBasis = simd_double3x3(simd_double3(matrix.columns.0.x, matrix.columns.0.y, matrix.columns.0.z),
                                simd_double3(matrix.columns.1.x, matrix.columns.1.y, matrix.columns.1.z),
                                simd_double3(matrix.columns.2.x, matrix.columns.2.y, matrix.columns.2.z))

        mOrigin = simd_double3(x: matrix.columns.3.x, y: matrix.columns.3.y, z: matrix.columns.3.z)
    }

    public init(_ quaternion: simd_quatd, _ origin: simd_double3 = simd_double3(repeating: 0.0)) {
        mBasis = simd_double3x3(quaternion)
        mOrigin = origin
    }

    public init(_ basis: simd_double3x3 = simd_double3x3(1.0), _ origin: simd_double3 = simd_double3(repeating: 0.0)) {
        mBasis = basis
        mOrigin = origin
    }

    public init(_ copyFrom: Transform) {
        mBasis = copyFrom.mBasis
        mOrigin = copyFrom.mOrigin
    }

    public func mult(_ t1: Transform, _ t2: Transform) -> Void {
        mBasis = t1.mBasis * t2.mBasis
        mOrigin = t1.transformVector(t2.mOrigin)
    }

    private func transformVector(_ vector: simd_double3) -> simd_double3 {
        let rot = mBasis.transpose
        return simd_double3(x: simd_dot(rot[0], vector) + mOrigin.x,
                            y: simd_dot(rot[1], vector) + mOrigin.y,
                            z: simd_dot(rot[2], vector) + mOrigin.z)
    }

    public static func * (_ t1: Transform, _ t2: Transform) -> Transform {
        return Transform(t1.mBasis * t2.mBasis, t1.transformVector(t2.mOrigin))
    }

    public static func * (_ transform: Transform, _ vector: simd_double3) -> simd_double3 {
        return transform.transformVector(vector)
    }

    public static func * (_ transform: Transform, _ quaternion: simd_quatd) -> simd_quatd {
        return transform.rotation * quaternion
    }

    public static func *= (_ transform: inout Transform, _ other: Transform) -> Void {
        transform.mBasis *= other.mBasis
        transform.mOrigin += other.mOrigin
    }

    public func setIdentity() -> Void {
        mBasis = simd_double3x3(1.0)
        mOrigin = simd_double3(repeating: 0.0)
    }

    public func inverse() -> Void {
        let inv = mBasis.transpose

        mBasis = inv
        mOrigin = inv * -mOrigin
    }
}

func vectorToTransform(v: simd_double3) -> Transform {
    let ray_vec = simd_normalize(-v)
    let ray_yaw_rot = simd_quatd(roll: 0.0, pitch: 0.0, yaw: atan2(ray_vec.y, ray_vec.x))
    let ray_new_x = simd_double3x3(ray_yaw_rot) * simd_double3(x: 1.0, y: 0.0, z: 0.0)
    let ray_pitch_rot = simd_quatd(roll: 0.0,
                                   pitch: atan2(-ray_vec.z, simd_dot(ray_vec, ray_new_x)),
                                   yaw: 0.0)

    return Transform(ray_yaw_rot * ray_pitch_rot, simd_double3())
}
