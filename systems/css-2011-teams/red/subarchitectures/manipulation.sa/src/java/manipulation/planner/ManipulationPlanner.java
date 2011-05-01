/**
 * @author Marc Hanheide (marc@hanheide.de)
 */

package manipulation.planner;

import cast.architecture.ManagedComponent;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ItemException;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.SimulateMoveToPose;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Rect2;
import cogx.Math.Sphere3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

public class ManipulationPlanner extends ManagedComponent {

}
