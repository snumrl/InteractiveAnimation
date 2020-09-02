import rnn.RNNModel as rm
import tensorflow as tf


def get_config(folder):
    c = get_config_impl(folder)
    c.label = folder
    return c
    
def get_config_impl(folder):
    if (folder.startswith("walk")):
        c = WalkConfig()
        c.X_DIMENSION = 2
        c.Y_DIMENSION = 136
        c.joint_len_weight = 0
        c.LAYER_KEEP_PROB = 0.9
        #c.foot_weight = 0
        #c.foot_slide_weight = 0
        c.RNN_SIZE = 512
        c.NUM_OF_LAYERS = 4
        return c
    
    return None

class JointConfig(rm.RNNConfig):
    def __init__(self):
        self.X_DIMENSION = 45
        self.Y_DIMENSION = 56
        self.y_weights = [0.0554, 0.0554, 0.0554, 2.7701, 2.7701, 2.7701, 0.5540, 0.5540, 0.5540, 0.0277, 0.0277, 0.0277, 0.5540, 0.0277, 0.0277, 0.0277, 2.7701, 2.7701, 2.7701, 0.0554, 0.0554, 0.0554, 0.0277, 2.7701, 2.7701, 2.7701, 0.5540, 0.5540, 0.5540, 0.0277, 0.0277, 0.0277, 0.5540, 0.0277, 0.0277, 0.0277, 2.7701, 2.7701, 2.7701, 0.0554, 0.0554, 0.0554, 0.0277, 2.7701, 2.7701, 2.7701, 2.7701, 2.7701, 2.7701, 0.1108, 0.1108, 0.1108, 0.1108, 0.1108, 0.1108, 2.7701]
        
    def error(self, x, prev_y, y, generated):
        loss = tf.reduce_mean(tf.square(y - generated)*self.y_weights)
        return loss, [loss]

class WalkConfig(rm.RNNConfig):
    def __init__(self):
        self.X_DIMENSION = 2
        self.Y_DIMENSION = 45
#         self.Y_DIMENSION = 63
        
    def gru_cell(self):
        cell = tf.contrib.rnn.GRUCell(self.RNN_SIZE)
        return cell
    
    def lstm_elu_cell(self):
        cell = tf.contrib.rnn.BasicLSTMCell(self.RNN_SIZE, forget_bias=0.8, activation=tf.nn.elu)
        return cell    
    
    def lstm_cell_init(self):
        initializer = tf.truncated_normal_initializer(mean=0, stddev=0.1);
        cell = tf.contrib.rnn.LSTMCell(self.RNN_SIZE, forget_bias=1, initializer=initializer)
#         cell = tf.contrib.rnn.LSTMCell(self.RNN_SIZE, forget_bias=1, activation=tf.nn.elu)
        return cell    
    
    def lstm_cell_init_relu(self):
        initializer = tf.truncated_normal_initializer(mean=0, stddev=0.1);
        cell = tf.contrib.rnn.LSTMCell(self.RNN_SIZE, forget_bias=1, initializer=initializer, activation=tf.nn.relu)
        return cell    
    
    def multi_input_model(self, batchSize, stepSize, lr=0.0001):
        return rm.MultiInputModel(self, batchSize, stepSize, lr)
    
    def state_model(self, batchSize, stepSize, lr=0.0001):
        return rm.StateModel(self, batchSize, stepSize, lr)   
    
class BasketTimeConfig(rm.RNNConfig):
    def __init__(self):
        self.X_DIMENSION = 11
        self.Y_DIMENSION = 45
        self.G_DIMENSION = 9
        self.E_DIMENSION = 1
    
    def model(self, batchSize, stepSize, lr=0.0001):
        return rm.TimeModel(self, batchSize, stepSize, lr)
    
    def runtime_model(self, batchSize):
        return rm.RuntimeModel(self, batchSize)
        
