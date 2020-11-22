from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2 
import rospy

traffic_light_state = ['Green', 'Yellow', 'Red', 'Unknown', 'Unknown']

class TLClassifier(object):
    def __init__(self, is_site):
        if is_site:
            PATH_TO_FROZEN_GRAPH = 'light_classification/model/site_model/'
        else:
            PATH_TO_FROZEN_GRAPH = 'light_classification/model/sim_model/'
        
        #Used frozen_inference_graph.pb based on mentor instructions
        FROZEN_GRAPH = PATH_TO_FROZEN_GRAPH + 'frozen_inference_graph.pb'
        rospy.logerr(FROZEN_GRAPH)

        self.graph = tf.Graph()
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                rospy.logerr("After parsing before Importing graph")
                tf.import_graph_def(od_graph_def, name='')
        
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.sess = tf.Session(graph=self.graph)


    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        
        return box_coords


    def draw_boxes(self, image, boxes, classes, scores):
        """Draw bounding boxes on the image"""
        for i in range(len(boxes)):
            top, left, bot, right = boxes[i, ...]
            cv2.rectangle(image, (left, top), (right, bot), (255,0,0), 3)
            text = traffic_light_state[int(classes[i])-1] + ': ' + str(int(scores[i]*100)) + '%'
            cv2.putText(image , text, (left, int(top - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,0,0), 1, cv2.LINE_AA)

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
        
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes


    def get_classification(self, image, is_site):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = np.dstack((image[:, :, 2], image[:, :, 1], image[:, :, 0]))
        if is_site:
            width = image.shape[1]
            height = image.shape[0]
            #rospy.loginfo("Width: %r, height: %r" % (width, height))
            image = image[:int(height/2), :, :]
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        with tf.Session(graph=self.graph) as sess:                
            # Actual detection.
            (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: image_np})
            # Dimension reduction for performance efficiency and simplicity
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
        
            confidence_cutoff = 0.7
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
        
        # Save boxed images
        write = True
        if write:
            image = np.dstack((image[:, :, 2], image[:, :, 1], image[:, :, 0]))
            width, height = image.shape[1], image.shape[0]
            box_coords = self.to_image_coords(boxes, height, width) 
            self.draw_boxes(image, box_coords, classes, scores)
            cv2.imwrite('/home/workspace/CarND-Capstone/imgs/img.jpg', image)
        
        
        if len(scores)>0:
            this_class = int(classes[np.argmax(scores)])
        else:
            this_class = 4
            
        if this_class == 1:
            return TrafficLight.GREEN
        elif this_class == 2:
             return TrafficLight.YELLOW
        elif this_class == 3:
             return TrafficLight.RED
                    
        return TrafficLight.UNKNOWN