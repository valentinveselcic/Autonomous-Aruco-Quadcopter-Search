from . import event


class signal(object):
    All = event.Event('*')


class dispatcher(object):
    def __init__(self):
        # dictionnary with signal as a key that contains a receiver (handler) list
        self.connection_list = {signal.All:[]}
    

    def connect(self,receiver, sig=signal.All):
        # Query the receiver associated with sig
        if sig in self.connection_list:
            # sig is already in the connection_list
            receivers = self.connection_list[sig]
        else:
            # create a new connection
            receivers = self.connection_list[sig] = []
            
        # add the receiver in the handler list associated with this signal
        receivers.append(receiver)

    def disconnect(self,receiver, sig=signal.All):
        if sig is signal.All:
            # disconnect this receiver from all the signals in the connection_list
            for sig in self.connection_list:
                # check if receiver is in the handler list of that signal
                if receiver in self.connection_list[sig]:
                    # remove it
                    self.connection_list[sig].remove(receiver)
                    
        elif sig in self.connection_list:
            # Query the handler list for that signal
            if receiver in self.connection_list[sig]:
                # remove the handler
                self.connection_list[sig].remove(receiver)


    def send(self,sig, **named):
        if sig in self.connection_list:
            # The signal is in the connection list
            # send to that signal and all
            receivers = self.connection_list[sig] + self.connection_list[signal.All]
        else:
            # the signal is not in the connection_list
            # only send to "all"
            receivers = self.connection_list[signal.All]
        
        for receiver in receivers:
            receiver(event=sig, **named)
            
            
if __name__=="__main__":
    def handler0(event, sender, **args):
        recvs.append(0)
        print('handler0: event=%s sender=%s' % (str(event), str(sender)))
        print(args)
    def handlerAll(event, sender, **args):
        recvs.append(0)
        print('handlerAll: event=%s sender=%s' % (str(event), str(sender)))
        print(args)

    # create some signals
    test_signal0 = event.Event('test signal0')
    test_signal1 = event.Event('test signal1')
    
    recvs=[] 
    dispatch=dispatcher()
    dispatch.connect(handler0, test_signal0)
    dispatch.send(test_signal0,sender='me')
    print('Send 0')
    dispatch.send(test_signal0,sender='me')
    print('Send 1')
    dispatch.send(test_signal1,sender='me')
    
    dispatch.connect(handlerAll, signal.All)
    print('Send 0')
    dispatch.send(test_signal0,sender='me')
    print('Send 1')
    dispatch.send(test_signal1,sender='me')

    print('Disconnect handler0')
    dispatch.disconnect(handler0, test_signal0)
    print('Send 0')
    dispatch.send(test_signal0,sender='me')
    print('Send 1')
    dispatch.send(test_signal1,sender='me')

    print('Disconnect handlerAll')
    dispatch.disconnect(handlerAll, test_signal0)
    print('Send 0')
    dispatch.send(test_signal0,sender='me')
    print('Send 1')
    dispatch.send(test_signal1,sender='me')

