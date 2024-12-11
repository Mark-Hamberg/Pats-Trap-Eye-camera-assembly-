
import time
import threading
import _thread

from DR_error import *
import queue
#from collections import deque

# thread state
TH_STATE_RUN = 1
TH_STATE_PAUSE = 2
TH_STATE_STOP = 3

TH_MAX = 4

_thread_error_Q = queue.Queue()


class DR_thread(threading.Thread):

    def __init__(self, func, loop=False):
        threading.Thread.__init__(self)
        # self.daemon = True

        self.__started = False
        self.__pause = False
        self.__stop = False
        self.__name = str(func)

        # set func
        self.func = func
        self.loop = loop

    def __exit__(self):
        print("_______thread {0} destoryed".format(self.__name))

    def run(self):
        try:
            self.__started = True

            while True:
                # pause
                while self.__pause == True:
                    if self.__stop == True:
                        break
                    time.sleep(0.01)

                # stop
                if True == self.__stop:
                    break

                # print("Thread process !!!")
                self.func()

                #DR_thread._thread_history.append(self.__name)
                ## print(DR_thread._thread_history)
                #
                #if th_new_id > 1:
                #    oldest_th = DR_thread._thread_history[0]
                #    for item in DR_thread._thread_history:
                #        if oldest_th != item:
                #            break
                #    else:
                #        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ time.sleep(0.1)")
                #        time.sleep(0.1)

                time.sleep(0.01)

                if self.loop == False:
                    self.__stop = True
                    break

        # stopped
        except KeyboardInterrupt as e:
            print("        >>>> KeyboardInterrupt (Stopped!!) in thread !!!!!! ", self.__name)

            from DRCF import release_lock
            release_lock()

        except (Exception, SystemExit) as e:
            print(">>>> Exception in thread !!!!!! ", self.__name)
            # print("e.msg ---> ", e.args[0])

            exc_type, exc_value, exc_traceback = sys.exc_info()

            traceback_details = {
                'filename': exc_traceback.tb_frame.f_code.co_filename,
                'lineno': exc_traceback.tb_lineno,
                'name': exc_traceback.tb_frame.f_code.co_name,
                'type': exc_type.__name__,
                # 'message': exc_value.message,  # or see traceback._some_str()
                'message': str(exc_value),  # or see traceback._some_str()
            }
            #print(traceback_details)

            # get line number
            lineno = 0
            lineno_temp = 0
            for frame in traceback.extract_tb(sys.exc_info()[2]):
                filename, lineno_temp, funcname, msg = frame

                if filename == "<string>":
                    lineno = lineno_temp
                    # break (not break to continue call stack)

            # set thread exception info
            THREAD_EXCEPT_INFO = [traceback_details['type'], lineno, traceback_details['message']]
            print(THREAD_EXCEPT_INFO)

            # clean thread
            self.__stop = True
            clean_thread() #thread except 가 발생하면 모든 쓰레드를 죽이도록 함 2017/08/30

            # interrupt_main()
            global _thread_error_Q

            _thread_error_Q.put(THREAD_EXCEPT_INFO)
            _thread.interrupt_main()

        finally:
            self.__stop = True
            print("        >>>> thread stopped... ", self.__name)

    def wait_started(self):
        while self.__started != True:
            time.sleep(0.01)
            pass

    def pause(self):
        self.__pause = True

    def resume(self):
        self.__pause = False

    def stop(self):
        self.__stop = True
        # print(">>>>>>>>>>>>>>>>>>>>> wait for thread finishing")

        start_time = time.time()
        while time.time() - start_time < 10:    # 10 second
            if self.is_alive():
                time.sleep(0.01)
            else:
                break
        else:
            print(">>>> The thread was not finished!! ... ", self.__name)
            raise RuntimeError  # when the thread is alive after 10 seconds, raise RuntimeError

        print(">>>> thread forced stopped ... ", self.__name)

    def state(self):
        if not self.is_alive():
            self.__stop = True
            return TH_STATE_STOP
        else:
            if self.__pause == True:
                return TH_STATE_PAUSE
            elif self.__stop == True:
                return TH_STATE_STOP
            else:
                return TH_STATE_RUN

# =============================================================================================

# thread list (max TH_MAX 개)
th_list = [0] * TH_MAX


def init_thread():
    global th_list
    global _thread_error_Q

    th_list = [0] * TH_MAX

    # empty queue
    while not _thread_error_Q.empty():
        _thread_error_Q.get()


def clean_thread():
    #print(">>>> clean thread()_______")
    print("         clean_thread() call")

    for id in range(0,TH_MAX):
        state = thread_state(id)

        if TH_STATE_STOP != state:
            print(">>>> thread_state({0}) = {1}".format(id, state))
            thread_stop(id)

    clear_thread_error_queue()


def get_thread_error_queue():
    global _thread_error_Q

    return _thread_error_Q


def clear_thread_error_queue():
    global _thread_error_Q

    while not _thread_error_Q.empty():
        _thread_error_Q.get()


def thread_run(th_func, loop=False) -> int:
    """
    This function creates and executes a thread. The features executed by the thread are determined by the functions
    specified in th_func_name.

    :param th_func: callable - Name of the function run by the thread.
    :param loop: bool - Flag indicates whether the thread will be repeated (True: Repeated calling of th_func_name(interval 0.01second), False: One-time calling of th_func_name)
    :return: Registered thread ID.
    """
    global th_list

    # th_func_name
    if callable(th_func) != True:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : th_func")

    # loop
    if type(loop) != bool:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : loop")

    for id  in range(0,TH_MAX):
        print(id)
        if th_list[id] == 0:
            th_list[id] = DR_thread(th_func, loop)
            th_list[id].start()
            th_list[id].wait_started()
            break
    else:
        raise DR_Error(DR_ERROR_RUNTIME, ("Cann't create thread (max thread = "+str(TH_MAX)+")"))

    #th_list[id].wait_started()

    print("__New thread [{0}, {1}] started!!".format(id, th_func.__name__))
    return id


def thread_stop(th_id) -> int:
    """
    This function terminates a thread. The program is automatically terminated when the DRL program is terminated even
    if the thread_stop() command is not used.

    :param th_id: int - Thread ID to stop
    :return: int - (0 -> Success, Negative value -> Error)
    """
    global th_list

    print("____try to stop thread {0}".format(th_id))

    # ra
    if type(th_id) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : th_id")

    if th_id < 0 or th_id >= TH_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value, th_id.")

    if th_list[th_id]:
        th_list[th_id].stop()
        th_list[th_id]=0

    return 0


def thread_pause(th_id):
    """
    This function temporarily suspends a thread.

    :param th_id: int - Thread ID to stop
    :return: int - (0 -> Success, Negative value -> Error)
    """
    global th_list

    # ra
    if type(th_id) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : th_id")

    if th_id < 0 or th_id >= TH_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value, th_id.")

    th_list[th_id].pause()

    return 0


def thread_resume(th_id):
    """
    This function resumes a temporarily suspended thread.

    :param th_id: int - Thread ID to stop
    :return: int - (0 -> Success, Negative value -> Error)
    """
    global th_list

    # ra
    if type(th_id) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : th_id")

    if th_id < 0 or th_id >= TH_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value, th_id.")

    th_list[th_id].resume()

    return 0


def thread_state(th_id):
    """
    This function checks the status of a thread.

    :param th_id: int - Thread ID to stop
    :return: TH_STATE_RUN, TH_STATE_PAUSE, TH_STATE_STOP
    """
    global th_list

    # ra
    if type(th_id) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : th_id")

    if th_id < 0 or th_id >= TH_MAX:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value, th_id.")

    if th_list[th_id]:
        return th_list[th_id].state()
    else:
        return TH_STATE_STOP

    return TH_STATE_STOP
