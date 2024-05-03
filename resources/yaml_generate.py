import random
import yaml

def generate_spawn_yaml(output_file_path):
    # Definitions
    types_available = ["battery", "pump", "regulator", "sensor"]
    colors_available = ["blue", "green", "red", "orange", "purple"]  # Extended color list for variety
    slots_range = list(range(1, 10))  # Slots from 1 to 9
    rotations = ['pi/6', 'pi/2', 'pi/3','pi/10','pi/9','pi/8','pi/7','pi/5','pi/4', '0', '-pi/6', '-pi/3', '-pi/2', '-pi/10','-pi/9','-pi/8','-pi/7','-pi/5','-pi/4']

    # Initialize bins inside a dictionary to accommodate the structure
    parts_structure = {"bins": {f"bin{i}": [] for i in range(1, 9)}}  # Adjust range based on the number of bins you want

    for bin_key in parts_structure["bins"].keys():
        available_slots = slots_range.copy()
        parts_to_spawn = random.randint(1, 5)  # Adjust range as needed; set to spawn 5 parts
        
        for _ in range(parts_to_spawn):
            selected_type = random.choice(types_available)
            selected_color = random.choice(colors_available)
            
            # Ensure num_slots does not exceed the number of available_slots
            max_slots = len(available_slots)
            if max_slots == 0:  # If no slots are available, skip to next bin
                break
            
            num_slots = random.randint(1, max(max_slots, 1))  # Ensure at least 1, up to available
            if num_slots < 5:
                num_slots = num_slots % 5
            slots_selected = random.sample(available_slots, k=num_slots)

            # Removing selected slots from available slots
            available_slots = [slot for slot in available_slots if slot not in slots_selected]

            part = {
                'type': selected_type,
                'color': selected_color,
                'slots': slots_selected,  # This ensures each slot is listed separately
                'rotation': random.choice(rotations)
            }
            parts_structure["bins"][bin_key].append(part)

    # Wrapping the bins structure under a top-level 'parts' key
    output_structure = {"parts": parts_structure}

    # Saving to a YAML file
    with open(output_file_path, 'w') as file:
        yaml.dump(output_structure, file, allow_unicode=True, sort_keys=False)
    print(f"YAML file generated at: {output_file_path}")

# Example usage
if __name__ == "__main__":
    for i in range(10,20):
        output_file_path = f'yamls/spawned_parts_{i}.yaml'  # Specify your output file path here
        generate_spawn_yaml(output_file_path)
