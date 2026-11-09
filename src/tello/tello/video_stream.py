import threading
from . protocol import *



class VideoStream(object):
    def __init__(self,LOGGER):
        self.closed = False
        self.cond = threading.Condition()
        self.queue = None
        self.current_frame=None
        self.name = 'VideoStream'
        self.closed = False
        self.LOGGER=LOGGER
        
    def read(self, size):
        #self.LOGGER.debug('%s.read with size %d Queue length : %d'%(self.name,size,len(self.queue)))
        with self.cond:
            #self.LOGGER.debug('  Cond acquired')
            while self.queue is None and self.current_frame is None and not self.closed:
                self.cond.wait(1)
            
            # check if we need to use self.current_frame
            if self.current_frame is None:
                self.current_frame=self.queue
                self.queue=None
                
            # We have some data to read
            data_to_read=size
            data=bytes()
            while self.current_frame is not None and data_to_read>0:
                # read self.current_frame
                if data_to_read>len(self.current_frame):
                    # read all data
                    data+=self.current_frame
                    data_to_read=data_to_read-len(self.current_frame)
                    self.current_frame=None
                else:
                    # read requested amount of data
                    data+=self.current_frame[:data_to_read]
                    self.current_frame=self.current_frame[data_to_read:]
                    data_to_read=0

                    
                # Transfer the queue as current frame
                if self.current_frame is None and self.queue is not None:
                    self.current_frame=self.queue
                    self.queue=None
            
            
        # returning data of zero length indicates end of stream
        #self.LOGGER.debug('  %s.read(size=%d) = %d' % (self.name, size, len(data)))
        return data

    def seek(self, offset, whence):
        self.LOGGER.info('%s.seek(%d, %d)' % (str(self.name), offset, whence))
        return -1
        
    def end_stream(self):
        self.LOGGER.debug('End stream****')
        self.cond.acquire()
        self.queue = bytes()
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()
        
    def update_raw_data(self, data):
        # discard unread frame to avoid accumulation in queue 
        # if the frame are not consumed
        self.cond.acquire()     
        self.queue=data
        self.cond.notifyAll()
        self.cond.release()
        #self.LOGGER.debug('VideoStream : update raw data, queue len %d'%(len(self.queue)))
        
        
        
        
        
