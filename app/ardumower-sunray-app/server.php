<?php

/*
  data sharing via numbers - put this on a PHP server and create a folder 'shared' and make it writable (chmod o+w)
*/
 
  
if (!isset($_POST["id"])) {
  exit();
}
    
$id = $_POST["id"];
if (!is_numeric($id)){
  exit();
}

if (isset($_POST["data"])) {
  $data = $_POST["data"];
  file_put_contents("./shared/$id.txt", $data);
} else {
  $data = "???";
  if (file_exists ( "./shared/$id.txt" )){
    $data = file_get_contents("./shared/$id.txt");  
  }  
  print($data);
}


?>
