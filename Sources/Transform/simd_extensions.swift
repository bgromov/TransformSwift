//
//  simd_extensions.swift
//  TransformSwift
//
//  Created by Boris Gromov on 29.06.2020.
//  

import Foundation
import simd

public extension simd_double4x4 {
    var diag: simd_double4 { simd_double4((0..<4).map { self[$0, $0] }) }

    init(_ m: simd_float4x4) {
        self.init(columns: (simd_double4(m.columns.0),
                            simd_double4(m.columns.1),
                            simd_double4(m.columns.2),
                            simd_double4(m.columns.3)))
    }
}

public extension simd_float4x4 {
    var diag: simd_float4 { simd_float4((0..<4).map { self[$0, $0] }) }

    init(_ m: simd_double4x4) {
        self.init(columns: (simd_float4(m.columns.0),
                            simd_float4(m.columns.1),
                            simd_float4(m.columns.2),
                            simd_float4(m.columns.3)))
    }
}

public extension simd_double3x3 {
    var diag: simd_double3 { simd_double3((0..<3).map { self[$0, $0] }) }

    /// Plain data in row-major order
    var rm_data: [Double] {
        return [columns.0[0], columns.1[0], columns.2[0],
                columns.0[1], columns.1[1], columns.2[1],
                columns.0[2], columns.1[2], columns.2[2]]
    }

    init(_ m: simd_float3x3) {
        self.init(columns: (simd_double3(m.columns.0),
                            simd_double3(m.columns.1),
                            simd_double3(m.columns.2)))
    }
}

public extension simd_float3x3 {
    var diag: simd_float3 { simd_float3((0..<3).map { self[$0, $0] }) }

    /// Plain data in row-major order
    var rm_data: [Float] {
        return [columns.0[0], columns.1[0], columns.2[0],
                columns.0[1], columns.1[1], columns.2[1],
                columns.0[2], columns.1[2], columns.2[2]]
    }

    init(_ m: simd_double3x3) {
        self.init(columns: (simd_float3(m.columns.0),
                            simd_float3(m.columns.1),
                            simd_float3(m.columns.2)))
    }
}

public extension simd_quatd {
    /// The x-component of the imaginary (vector) part.
    var x: Double { imag.x }
    /// The y-component of the imaginary (vector) part.
    var y: Double { imag.y }
    /// The z-component of the imaginary (vector) part.
    var z: Double { imag.z }
    /// The real (scalar) part.
    var w: Double { real }

    var rpy: (roll: Double, pitch: Double, yaw: Double) {
        get {
            let mat: simd_double3x3 = simd_double3x3(self)

            var yaw = Double.nan
            var pitch = Double.nan
            var roll = Double.nan

            pitch = atan2(-mat.rm_data[6], sqrt((mat.rm_data[0] * mat.rm_data[0] + mat.rm_data[3] * mat.rm_data[3])))

            if fabs(pitch) > (Double.pi / 2 - Double.ulpOfOne) {
                yaw  = atan2(-mat.rm_data[1], mat.rm_data[4])
                roll = 0.0
            } else {
                roll = atan2(mat.rm_data[7], mat.rm_data[8])
                yaw  = atan2(mat.rm_data[3], mat.rm_data[0])
            }

            return (roll: roll, pitch: pitch, yaw: yaw)
        }
    }

    init(roll: Double = 0.0, pitch: Double = 0.0, yaw: Double = 0.0) {
        let hy = yaw / 2.0
        let hp = pitch / 2.0
        let hr = roll / 2.0

        let cy = cos(hy)
        let sy = sin(hy)
        let cp = cos(hp)
        let sp = sin(hp)
        let cr = cos(hr)
        let sr = sin(hr)

        let quat: simd_double4 =
            simd_double4(x: sr * cp * cy - cr * sp * sy,
                         y: cr * sp * cy + sr * cp * sy,
                         z: cr * cp * sy - sr * sp * cy,
                         w: cr * cp * cy + sr * sp * sy)

        self.init(vector: quat)
    }
}

public extension simd_quatf {
    /// The x-component of the imaginary (vector) part.
    var x: Float { imag.x }
    /// The y-component of the imaginary (vector) part.
    var y: Float { imag.y }
    /// The z-component of the imaginary (vector) part.
    var z: Float { imag.z }
    /// The real (scalar) part.
    var w: Float { real }

    var rpy: (roll: Float, pitch: Float, yaw: Float) {
        get {
            let mat: simd_float3x3 = simd_float3x3(self)

            var yaw = Float.nan
            var pitch = Float.nan
            var roll = Float.nan

            pitch = atan2(-mat.rm_data[6], sqrt((mat.rm_data[0] * mat.rm_data[0] + mat.rm_data[3] * mat.rm_data[3])))

            if abs(pitch) > (Float.pi / 2 - Float.ulpOfOne) {
                yaw  = atan2(-mat.rm_data[1], mat.rm_data[4])
                roll = 0.0
            } else {
                roll = atan2(mat.rm_data[7], mat.rm_data[8])
                yaw  = atan2(mat.rm_data[3], mat.rm_data[0])
            }

            return (roll: roll, pitch: pitch, yaw: yaw)
        }
    }

    init(roll: Float = 0.0, pitch: Float = 0.0, yaw: Float = 0.0) {
        let hy = yaw / 2.0
        let hp = pitch / 2.0
        let hr = roll / 2.0

        let cy = cos(hy)
        let sy = sin(hy)
        let cp = cos(hp)
        let sp = sin(hp)
        let cr = cos(hr)
        let sr = sin(hr)

        let quat: simd_float4 =
            simd_float4(x: sr * cp * cy - cr * sp * sy,
                         y: cr * sp * cy + sr * cp * sy,
                         z: cr * cp * sy - sr * sp * cy,
                         w: cr * cp * cy + sr * sp * sy)

        self.init(vector: quat)
    }
}

public extension String.StringInterpolation {
    mutating func appendInterpolation(_ v: simd_float3x3, _ width: Int = 8) {
        var str: String = "["
        for i in 0..<v.rm_data.count {
            if (i + 1) % 3 == 1 {
                if i == 0 {
                    str += "["

                } else {
                    str += String(repeating: " ", count: description.count + 1)
                    str += "["
                }
            }
            str += String(format: "%\(width).2g", v.rm_data[i])
            if (i + 1) % 3 == 0 {
                if (i + 1) != v.rm_data.count {
                    str += "],\n"
                } else {
                    str += "]\n"
                }
            } else {
                str += ", "
            }
        }
        str = str.trimmingCharacters(in: .controlCharacters) + "]"
        appendInterpolation(str)
    }

    mutating func appendInterpolation(_ v: simd_double3x3, _ width: Int = 8) {
        var str: String = "["
        for i in 0..<v.rm_data.count {
            if (i + 1) % 3 == 1 {
                if i == 0 {
                    str += "["

                } else {
                    str += String(repeating: " ", count: description.count + 1)
                    str += "["
                }
            }
            str += String(format: "%\(width).2g", v.rm_data[i])
            if (i + 1) % 3 == 0 {
                if (i + 1) != v.rm_data.count {
                    str += "],\n"
                } else {
                    str += "]\n"
                }
            } else {
                str += ", "
            }
        }
        str = str.trimmingCharacters(in: .controlCharacters) + "]"
        appendInterpolation(str)
    }
}

public extension SIMD where Scalar: Numeric {
    var flat: [Scalar] { indices.map{ self[$0] } }

    func str(_ scalarFormat: String = "%3.2g") -> String {
        let vals = indices.map{ self[$0] }
        let strs = vals.map { String(format: scalarFormat, $0 as! CVarArg) }
        return "[" + strs.joined(separator: ", ") + "]"
    }
}
