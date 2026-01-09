# --------------------------------------------------
# Fill right_lane and left_lane from counterfactual sets
# --------------------------------------------------

library(readr)
library(dplyr)

# -----------------------------
# Configuration
# -----------------------------

input_file  <- "counterfactuals_combinations.csv"
output_file <- "counterfactuals_combinations_random_choices.csv"

action_cols <- c(
  "swerve_left",
  "swerve_right",
  "cruise",
  "keep",
  "change_to_left",
  "change_to_right"
)

set.seed(123)  # remove if you want true randomness

# -----------------------------
# Read data
# -----------------------------

df <- read_csv(input_file, show_col_types = FALSE)

# -----------------------------
# Helper: sample one valid action
# -----------------------------

pick_action <- function(row) {
  valid <- action_cols[row[action_cols] == 1]
  if (length(valid) == 0) {
    return(NA_character_)
  }
  sample(valid, 1)
}

# -----------------------------
# Fill both lane columns
# -----------------------------

df_filled <- df %>%
  rowwise() %>%
  mutate(
    right_lane = pick_action(cur_data()),
    left_lane  = pick_action(cur_data())
  ) %>%
  ungroup()

# -----------------------------
# Save output
# -----------------------------

write_csv(df_filled, output_file)

message("Done. Output written to: ", output_file)

