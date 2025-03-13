document.addEventListener("DOMContentLoaded", () => {
    let count = 0;
    const button = document.getElementById("counterButton");
    const display = document.getElementById("counter");

    button.addEventListener("click", () => {
        count++;
        display.textContent = count;
    });
});
