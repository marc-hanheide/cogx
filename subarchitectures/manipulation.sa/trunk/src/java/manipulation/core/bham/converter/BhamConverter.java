package manipulation.core.bham.converter;

import golem.tinyice.Mat33;
import golem.tinyice.Vec3;

import java.util.LinkedList;
import java.util.List;

import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.ModelPoint;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.VisionModel;

import org.apache.commons.math.linear.Array2DRowRealMatrix;

import VisionData.GeometryModel;
import VisionData.Vertex;
import cogx.Math.Matrix33;

//TODO in factory einbauen?!

/**
 * represents a collection of convert functions
 * 
 * @author ttoenige
 * 
 */
public class BhamConverter {

	/**
	 * convert a matrix to the GOLEM matrix representation
	 * 
	 * @param matrix
	 *            matrix to convert
	 * @return GOLEM matrix representation
	 */
	public static Mat33 convMatrixToGolem(Matrix matrix) {
		return new Mat33(matrix.getM00(), matrix.getM01(), matrix.getM02(),
				matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix
						.getM20(), matrix.getM21(), matrix.getM22());
	}

	/**
	 * convert a GOLEM matrix to a matrix representation
	 * 
	 * @param matrix
	 *            GOLEM matrix to convert
	 * @return matrix representation
	 */
	public static Matrix convGolemToMatrix(Mat33 matrix) {
		return new Matrix(matrix.m11, matrix.m21, matrix.m31, matrix.m12,
				matrix.m22, matrix.m32, matrix.m13, matrix.m23, matrix.m33);
	}

	/**
	 * convert a GOLEM vector representation to a vector
	 * 
	 * @param vector
	 *            GOLEM vector representation to convert
	 * @return vector representation
	 */
	public static Vector3D convGolemToVec(Vec3 vector) {
		return new Vector3D(vector.v1, vector.v2, vector.v3);
	}

	/**
	 * convert a vector to a GOLEM vector representation
	 * 
	 * @param vector
	 *            vector representation to convert
	 * @return GOLEM vector representation
	 */
	public static Vec3 convVecToGolem(Vector3D vector) {
		return new Vec3(vector.getX(), vector.getY(), vector.getZ());
	}

	/**
	 * convert a BLORT matrix representation to a matrix
	 * 
	 * @param matrix
	 *            BLORT matrix representation to convert
	 * @return matrix representation
	 */
	public static Matrix convBlortToMatrix(Matrix33 matrix) {
		return new Matrix(matrix.m00, matrix.m10, matrix.m20, matrix.m01,
				matrix.m11, matrix.m21, matrix.m02, matrix.m12, matrix.m22);
	}

	/**
	 * convert BLORT geometric model representation of an item to vision model
	 * representation
	 * 
	 * @param geomModel
	 *            BLORT geometric model representation to convert
	 * @return vision meodel representation
	 */
	public static VisionModel convBlortGeomModelToVisionModel(
			GeometryModel geomModel) {
		List<ModelPoint> modelPoints = new LinkedList<ModelPoint>();

		for (Vertex vertex : geomModel.vertices) {
			ModelPoint newPoint = new ModelPoint(new Vector3D(vertex.pos.x,
					vertex.pos.y, vertex.pos.z), new Vector3D(vertex.normal.x,
					vertex.normal.y, vertex.normal.z));
			modelPoints.add(newPoint);
		}

		VisionModel model = new VisionModel(modelPoints);

		return model;
	}

	/**
	 * convert a vector to a Array2DRowRealMatrix representation
	 * 
	 * @param point
	 *            vector to convert
	 * @return Array2DRowRealMatrix representation of the vector
	 */
	public static Array2DRowRealMatrix convertVecToRowMatrix(Vector3D point) {
		double[][] matrix = new double[3][1];
		matrix[0][0] = point.getX();
		matrix[1][0] = point.getY();
		matrix[2][0] = point.getZ();

		return new Array2DRowRealMatrix(matrix);
	}

}
