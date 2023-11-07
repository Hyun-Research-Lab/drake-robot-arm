from pydrake.systems.framework import LeafSystem, EventStatus
from pydrake.all import ( AbstractValue)
import copy
import csv
class AbstractValueLogger(LeafSystem):
    def __init__(self, model_value, publish_period_seconds: float):
        super().__init__()
        self.DeclarePeriodicPublishEvent(publish_period_seconds, 0, self.Publish)
        self.abstract_value_input_port = self.DeclareAbstractInputPort(
            'abstract_value', AbstractValue.Make(model_value))
        self.sample_times = []
        self.values = []

    def Publish(self, context):
        self.sample_times.append(context.get_time())
        self.values.append(
            copy.deepcopy(self.abstract_value_input_port.Eval(context)))
        
    def WriteCSV(self, file_name, process_function=None):
        fieldnames = ['time', 'value']
        with open(file_name, 'w') as f:
            writer = None
            if process_function is not None:
                for v in self.values:
                    processed_value = process_function(v)
                    if writer is None:
                        fieldnames = ['time'] + list(processed_value.keys())
                        writer = csv.DictWriter(f, fieldnames=fieldnames)
                        writer.writeheader()
                    writer.writerow(processed_value)
            else:
                for i in range(len(self.sample_times)):
                    f.write(str(self.sample_times[i]) + ',')
                    f.write(str(self.values[i]) + '\n')

    #Example process_function
    '''
    def process_externally_applied_spatial_force(value):
        # Process the ExternallyAppliedSpatialForce object to a dictionary.
        # This is just a placeholder; you'll need to fill in the actual logic.
        return {
            'time': value.time,
            'body_index': value.body_index,
            # Convert vectors to list or tuple before writing to CSV.
            'p_BoBq_W': value.p_BoBq_W.tolist() if hasattr(value, 'p_BoBq_W') else "N/A",
            'F_Bq_W': value.F_Bq_W.tolist() if hasattr(value, 'F_Bq_W') else "N/A",
            'tau_BoBq_W': value.tau_BoBq_W.tolist() if hasattr(value, 'tau_BoBq_W') else "N/A",
        }
    '''