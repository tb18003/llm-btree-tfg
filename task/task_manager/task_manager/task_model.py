from json.decoder import JSONDecoder

from py_trees.behaviour import Behaviour

class Task(Behaviour):
    """
    ## Task

    This is a wrapper class of `Behaviour` class, adding task arguments in a class argument and a function which
    returns the behavior when task fail.
    """

    def __init__(self, args, name):
        super(Task, self).__init__(name)
        self.args = args

    def opposite_behavior(self) -> Behaviour | None:
        """
        This function returns the behavior that will follow the behavior tree, if the task fail. 
        Remember that the returned behavior **should be always succesful**. This function must be overrided by class which inherite `Task`.

        Returns:
            behavior(Behaviour | None): The behavior will follow the behavior tree if initial task fail.
        """
        return None

class TaskFactory:
    """
    ## Task Factory

    This class is a task creator based on the dictionary given in the constructor of the class. 

    Note: `Behavior` class belongs to package `py_trees.behavior`, and for our context, we will name this class
    as **Task** on documents.

    Attributes:
        implementations (dict[str, Behavior]): Dictionary of the children classes of <code>Behavior</code>, which key is identifier name and value is the class.
    """

    def __init__(self, actions: dict[str, Task]):
        self.actions = actions

    def parse_json_tasks(self, json : str) -> list[Task]:
        """
        Parse a JSON text and create one or mores instance of a task.

        Arguments:
            json(str): A JSON document which contains one or more tasks.
        
        Returns:
            task(list[Behavior]): A list with the instance of the tasks.
        
        Raises:
            Exception: It raises and exception if the task is not implemented on the system
        
        """
        obj = JSONDecoder().decode(json)

        if type(obj) != list:
            obj = [obj]
        
        tasks : list[Behaviour] = []

        for o in obj:
            if o["task"] in self.actions.keys():
                tasks.append(self.actions[o["task"]](o["args"] if o['args'] is not None else {}))
            else:
                print("[WARN] Cannot find task '" + o["task"] + "', ignoring task")
        
        return tasks
    
    @staticmethod
    def parse_json_tasks(actions: dict[str, Task],json : str) -> list[Task]:
        """
        Parse a JSON text and create one or mores instance of a task. 

        Arguments:
            actions(dict[str, Behavior]): Dictionary between implementations and 
            json(str): A JSON document which contains one or more tasks.
        
        Returns:
            task(list[Behavior]): A list with the instance of the tasks.
        
        Raises:
            Exception: It raises and exception if the task is not implemented on the system
        
        """
        obj = JSONDecoder().decode(json)

        if type(obj) != list:
            obj = [obj]
        
        tasks : list[Task] = []

        for o in obj:
            if o["task"] in actions.keys():
                tasks.append(actions[o["task"]](o["args"] if "args" in o.keys() is not None else {}))
            else:
                print("[TaskFactory :: WARN] Cannot find task '" + o["task"] + "', ignoring task")
        
        return tasks
    
