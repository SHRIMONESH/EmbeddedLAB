<?php
// --- SET TIMEZONE: THIS IS THE CRITICAL FIX ---
date_default_timezone_set('Asia/Kolkata');

require_once 'db_connect.php'; 

header("Content-Type: application/json");

// --- CONFIGURATION ---
$otp_validity_seconds = 60; // The OTP will be valid for 60 seconds

// Check if the RFID UID was sent
if (isset($_GET['uid'])) {
    $uid = mysqli_real_escape_string($conn, $_GET['uid']); 
    
    // Prepare a statement to get the existing OTP and its timestamp
    $stmt = $conn->prepare("SELECT otp, otp_timestamp FROM users WHERE rfid_uid = ?");
    $stmt->bind_param("s", $uid);
    $stmt->execute();
    $result = $stmt->get_result();
    
    if ($result->num_rows > 0) {
        $row = $result->fetch_assoc();
        $existing_otp = $row['otp'];
        $timestamp = $row['otp_timestamp'];

        // Calculate the time difference in seconds
        $time_diff = time() - strtotime($timestamp);

        // Check if a valid, non-expired OTP already exists
        if ($existing_otp && $timestamp && $time_diff < $otp_validity_seconds) {
            
            // A valid OTP exists. Return the EXISTING one.
            echo json_encode(['status' => 'success', 'otp' => $existing_otp]);

        } else {
            
            // No valid OTP exists (either it's null, or it has expired).
            // Generate a NEW OTP.
            $new_otp = rand(100000, 999999);
            
            // Prepare a statement to UPDATE the database with the new OTP and current timestamp
            $stmt_update = $conn->prepare("UPDATE users SET otp = ?, otp_timestamp = NOW() WHERE rfid_uid = ?");
            $stmt_update->bind_param("ss", $new_otp, $uid);
            
            if ($stmt_update->execute()) {
                // If the update was successful, send the NEW OTP back
                echo json_encode(['status' => 'success', 'otp' => $new_otp]);
            } else {
                echo json_encode(['status' => 'error', 'message' => 'Database update failed.']);
            }
            $stmt_update->close();
        }
    } else {
        echo json_encode(['status' => 'error', 'message' => 'Invalid RFID UID.']);
    }
    $stmt->close();

} else {
    echo json_encode(['status' => 'error', 'message' => 'RFID UID not provided.']);
}

$conn->close();
?>