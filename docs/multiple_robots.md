# Instructions for Using Multiple Robots

You can run **two robots simultaneously** by starting **two separate instances** of `main_loop`, each linked to a specific robot ID.

### Robot IDs
The robots follow a predefined ID pattern:
- `robot_1`
- `robot_2`

### Running `main_loop`
Use the `ID` flag to specify which robot to run:

```bash
python main_loop [ID]
```

**Default**: If no ID is provided, main_loop will run with robot_1, which is convenient for single-robot usage.

To run the second robot:

```bash
python main_loop robot_2
```

Robot settings are defined in the `.env` file.

> ⚠️ **Important:** If you have two robots of different models or want to use different configurations, follow these steps:
>
> 1. Run the first `main_loop` with the desired `.env` configuration.
> 2. Update the `.env` file for the second robot.
> 3. Run the second `main_loop`.
>
> This ensures that each robot runs with the correct configuration.