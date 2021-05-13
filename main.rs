use ndarray::arr2;

fn main() {
    let a = arr2(&[[1., 2, 3], [4, 5, 6]]);

    let b = arr2(&[[6., 5, 4],
                   [3, 2, 1]]);

    let sum = &a + &b;

    println!("{}", a);
    println!("+");
    println!("{}", b);
    println!("=");
    println!("{}", sum);

    let a1 = arr2(&[[1., 2, 3],
                   [4, 5, 6],
                    [1, 1, 1]]);

    let b1 = arr2(&[[6., 5, 4],
                   [3, 2, 1],
                    [1,1 ,1]]);
    let mul = a1.dot(&b1);
    println!("{}", mul)
}